#!/usr/bin/env python3
"""
Complete DH -> FK -> Analytic IK -> URDF -> PyBullet pipeline
with on-demand download of prebuilt URDFs (only once if missing).
Supports DOF 1-6 interactive entry. Prebuilt URDFs used for DOF 2,3,4,6.
"""
import numpy as np
from math import radians, degrees, pi, sin, cos, atan2, sqrt, acos
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import pybullet as p
import pybullet_data
import time
import logging
import csv
import os
import urllib.request
import pandas as pd
import shutil
logging.basicConfig(level=logging.INFO)
# -------------------------
# Utilities: DH, rotations
# -------------------------
def dh_transform(a, alpha, d, theta):
    ct, st = cos(theta), sin(theta)
    ca, sa = cos(alpha), sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ], dtype=float)
def rpy_to_rot(roll, pitch, yaw):
    cr, sr = cos(roll), sin(roll)
    cp, sp = cos(pitch), sin(pitch)
    cy, sy = cos(yaw), sin(yaw)
    Rz = np.array([[cy, -sy, 0],
                   [sy, cy, 0],
                   [0, 0, 1]])
    Ry = np.array([[cp, 0, sp],
                   [0, 1, 0],
                   [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr, cr]])
    return Rz @ Ry @ Rx
def rot_matrix_to_rpy(R):
    sy = sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        x = atan2(R[2, 1], R[2, 2])
        y = atan2(-R[2, 0], sy)
        z = atan2(R[1, 0], R[0, 0])
    else:
        x = atan2(-R[1, 2], R[1, 1])
        y = atan2(-R[2, 0], sy)
        z = 0.0
    return x, y, z
def rot_to_angle_deg(R_err):
    tr = np.trace(R_err)
    val = (tr - 1.0) / 2.0
    val = max(-1.0, min(1.0, val))
    angle = acos(val)
    return degrees(angle)
# -------------------------
# Link & Robot classes
# -------------------------
class Link:
    def __init__(self, a, alpha, d, theta, joint_type, q_min=None, q_max=None, radius=0.04, mass=1.0, color=None):
        self.a = float(a)
        self.alpha = float(alpha)
        self.d = float(d)
        self.theta = float(theta)
        self.joint_type = joint_type.upper()
        self.q_min = q_min
        self.q_max = q_max
        self.radius = float(radius)
        self.mass = float(mass)
        if color is None:
            self.color = (0.3, 0.6, 0.9, 1.0)
        else:
            if len(color) == 3:
                self.color = (color[0], color[1], color[2], 1.0)
            else:
                self.color = color
                #lohith
class Robot:
    def __init__(self, links):
        self.links = links
        self.n = len(links)
    def fk(self, q):
        T = np.eye(4)
        for i, link in enumerate(self.links):
            if link.joint_type == 'R':
                theta = q[i]
                d = link.d
            else:
                theta = link.theta
                d = q[i]
            T = T @ dh_transform(link.a, link.alpha, d, theta)
        return T
    def fk_all(self, q):
        Ts = [np.eye(4)]
        T = np.eye(4)
        for i, link in enumerate(self.links):
            if link.joint_type == 'R':
                theta = q[i]
                d = link.d
            else:
                theta = link.theta
                d = q[i]
            T = T @ dh_transform(link.a, link.alpha, d, theta)
            Ts.append(T.copy())
        return Ts
    def plot(self, q):
        Ts = self.fk_all(q)
        pts = np.array([T[:3, 3] for T in Ts])
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], 'o-', linewidth=3, color='k')
        for i, T in enumerate(Ts):
            o = T[:3, 3]
            x_ax, y_ax, z_ax = T[:3, 0], T[:3, 1], T[:3, 2]
            L = 0.1 + 0.02 * i
            ax.quiver(*o, *(x_ax * L), length=L, normalize=True)
            ax.quiver(*o, *(y_ax * L), length=L, normalize=True)
            ax.quiver(*o, *(z_ax * L), length=L, normalize=True)
            ax.text(*(o + x_ax * L * 1.2), f'F{i}', color='k')
        max_range = np.max(np.ptp(pts, axis=0)) + 0.2
        mid = np.mean(pts, axis=0)
        ax.set_xlim(mid[0] - max_range / 2, mid[0] + max_range / 2)
        ax.set_ylim(mid[1] - max_range / 2, mid[1] + max_range / 2)
        ax.set_zlim(mid[2] - max_range / 2, mid[2] + max_range / 2)
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        ax.set_title('Robot Pose (DH frames)')
        plt.show()
    def generate_urdf(self, filename='robot.urdf'):
        urdf = '<?xml version="1.0"?>\n<robot name="dh_robot">\n'
        urdf += '  <link name="base_link"/>\n'
        urdf += ('  <joint name="base_joint" type="fixed">\n'
                 '    <parent link="base_link"/>\n'
                 '    <child link="link0"/>\n'
                 '    <origin xyz="0 0 0" rpy="0 0 0"/>\n'
                 '  </joint>\n')
        for i, link in enumerate(self.links):
            length = max(abs(link.a), abs(link.d), 0.05)
            r, g, b, a = (*link.color[:3], 1.0)
            m = float(link.mass)
            Ixx = Iyy = (1.0 / 12.0) * m * (3 * (link.radius ** 2) + length ** 2)
            Izz = 0.5 * m * link.radius ** 2
            urdf += f'  <link name="link{i}">\n'
            urdf += f'    <visual>\n      <geometry><cylinder radius="{link.radius:.6f}" length="{length:.6f}"/></geometry>\n'
            urdf += f'      <origin xyz="0 0 {length / 2:.6f}" rpy="0 0 0"/>\n'
            urdf += f'      <material name="color{i}">\n        <color rgba="{r:.6f} {g:.6f} {b:.6f} {a:.6f}"/>\n      </material>\n    </visual>\n'
            urdf += f'    <collision>\n      <geometry><cylinder radius="{link.radius:.6f}" length="{length:.6f}"/></geometry>\n'
            urdf += f'      <origin xyz="0 0 {length / 2:.6f}" rpy="0 0 0"/>\n    </collision>\n'
            urdf += f'    <inertial>\n      <mass value="{m:.6f}"/>\n'
            urdf += f'      <origin xyz="0 0 {length / 2:.6f}" rpy="0 0 0"/>\n'
            urdf += (f'      <inertia ixx="{Ixx:.8f}" iyy="{Iyy:.8f}" izz="{Izz:.8f}" '
                     f'ixy="0" ixz="0" iyz="0"/>\n')
            urdf += '    </inertial>\n  </link>\n'
            if i < self.n - 1:
                jtype = "revolute" if link.joint_type == "R" else "prismatic"
                lower = link.q_min if link.q_min is not None else -pi
                upper = link.q_max if link.q_max is not None else pi
                urdf += f'  <joint name="joint{i+1}" type="{jtype}">\n'
                urdf += f'    <parent link="link{i}"/>\n'
                urdf += f'    <child link="link{i+1}"/>\n'
                urdf += f'    <origin xyz="0 0 {link.d:.6f}" rpy="{link.alpha:.6f} 0 0"/>\n'
                urdf += f'    <axis xyz="0 0 1"/>\n'
                urdf += f'    <limit lower="{float(lower):.6f}" upper="{float(upper):.6f}"/>\n'
                urdf += '  </joint>\n'
            else:
                urdf += ('  <link name="end_effector"/>\n'
                         '  <joint name="end_effector_fixed" type="fixed">\n'
                         f'    <parent link="link{i}"/>\n                         '
                         '    <child link="end_effector"/>\n'
                         f'    <origin xyz="0 0 {length:.6f}" rpy="0 0 0"/>\n'
                         '  </joint>\n')
        urdf += '</robot>\n'
        with open(filename, 'w') as f:
            f.write(urdf)
        logging.info(f"URDF saved to {filename}")
    def run_sim(self, q, use_prebuilt=False, prebuilt_urdf_path=None,
                save_jointmap=True, save_traj_csv=True, traj_csv_path='trajectory.csv'):
        physicsClient = p.connect(p.GUI)
        if physicsClient < 0:
            raise RuntimeError("Failed to connect to PyBullet!")
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1 / 240)
        p.setRealTimeSimulation(0)
        p.loadURDF("plane.urdf")
        if use_prebuilt and prebuilt_urdf_path is not None:
            robot_id = p.loadURDF(prebuilt_urdf_path, useFixedBase=True)
        else:
            # generate URDF to a local filename and load it
            gen_path = prebuilt_urdf_path or 'robot.urdf'
            self.generate_urdf(gen_path)
            robot_id = p.loadURDF(gen_path, useFixedBase=True)
        num_joints = p.getNumJoints(robot_id)
        movable_joints = []
        joint_info_map = {}
        for j in range(num_joints):
            info = p.getJointInfo(robot_id, j)
            jtype = info[2]
            name = info[1].decode('utf-8')
            if jtype in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                movable_joints.append(j)
                joint_info_map[len(movable_joints) - 1] = {'pybullet_index': j, 'name': name, 'type': 'revolute' if jtype == p.JOINT_REVOLUTE else 'prismatic'}
        if save_jointmap:
            with open('joint_map.txt', 'w') as mf:
                mf.write("index_in_q -> pybullet_joint_index, joint_name, joint_type\n")
                for idx, info in joint_info_map.items():
                    mf.write(f"{idx} -> {info['pybullet_index']}, {info['name']}, {info['type']}\n")
            logging.info("Joint map saved to joint_map.txt")
            print("Joint mapping (q[idx] -> pybullet_index, name, type):")
            for idx, info in joint_info_map.items():
                print(f"  q[{idx}] -> joint_index {info['pybullet_index']}, name='{info['name']}', type={info['type']}")
        num_movable = len(movable_joints)
        q = np.asarray(q).flatten()
        if q.size < num_movable:
            logging.warning("Provided q has fewer values than movable joints; padding with zeros.")
            target_q = np.zeros(num_movable)
            target_q[:q.size] = q
        else:
            target_q = q[:num_movable]
        q_start = np.zeros(num_movable)
        steps = 300
        trajectory = [q_start + (target_q - q_start) * t / steps for t in range(steps + 1)]
        if save_traj_csv:
            with open(traj_csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['time_step'] + [f'joint_{i}' for i in range(num_movable)])
                for t_idx, q_row in enumerate(trajectory):
                    writer.writerow([t_idx] + [float(v) for v in q_row])
            logging.info(f"Trajectory saved to {traj_csv_path}")
        logging.info("Starting PyBullet animation. Close window to stop.")
        try:
            for q_step in trajectory:
                for idx, pb_index in enumerate(movable_joints):
                    desired = float(q_step[idx])
                    p.setJointMotorControl2(robot_id, pb_index, p.POSITION_CONTROL, targetPosition=desired, force=200)
                p.stepSimulation()
                time.sleep(1 / 240)

            while p.isConnected():
                p.stepSimulation()
                time.sleep(1 / 240)
        except Exception as e:
            logging.error(f"PyBullet error: {e}")
        finally:
            if p.isConnected():
                p.disconnect()
                #anvesh
# -------------------------
# Analytic-only IKSolver
# -------------------------
class IKSolver:
    def __init__(self, robot):
        self.robot = robot
        self.n = robot.n
    def joint_pattern(self):
        return ''.join([lk.joint_type for lk in self.robot.links])
    def _alphas(self, count):
        return [self.robot.links[i].alpha for i in range(min(count, self.n))]
    def _is_planar_first_n(self, n_check):
        if self.n < n_check:
            return False
        for i in range(n_check):
            if self.robot.links[i].joint_type != 'R':
                return False
            if abs(self.robot.links[i].alpha) > 1e-3:  # alpha must be ≈ 0 for planar
                return False
        return True
    def _is_spherical_wrist(self):
        if self.n < 6:
            return False
        for i in range(self.n - 3, self.n):
            if self.robot.links[i].joint_type != 'R':
                return False
        l4, l5 = self.robot.links[self.n - 3], self.robot.links[self.n - 2]
        if abs(l4.a) < 1e-3 and abs(l5.a) < 1e-3:
            return True
        alphas = [abs(self.robot.links[self.n - 3].alpha), abs(self.robot.links[self.n - 2].alpha), abs(self.robot.links[self.n - 1].alpha)]
        if any(abs(a - pi / 2) < 0.3 for a in alphas):
            return True
        return False
    # Analytic solvers (PPP, RRP, PRP, 2R, 3R, 6R spherical wrist)
    def analytic_cartesian_ppp(self, target_pos):
        if self.n < 3:
            raise RuntimeError("PPP requires at least 3 prismatic joints.")
        if not all(self.robot.links[i].joint_type == 'P' for i in range(3)):
            raise RuntimeError("PPP requires first three joints P-P-P.")
        px, py, pz = target_pos
        q = np.zeros(self.n)
        q[0], q[1], q[2] = px, py, pz
        print("\n--- Cartesian PPP analytic solution ---")
        print(f"q1 = {q[0]:.6f}, q2 = {q[1]:.6f}, q3 = {q[2]:.6f}")
        print("--------------------------------------\n")
        return [q]
    def analytic_scara_rrp(self, target_pos):
        if self.n < 3:
            raise RuntimeError("SCARA R-R-P requires at least 3 joints.")
        if not (self.robot.links[0].joint_type == 'R' and self.robot.links[1].joint_type == 'R' and self.robot.links[2].joint_type == 'P'):
            raise RuntimeError("SCARA R-R-P requires first two R, third P.")
        a1 = self.robot.links[0].a
        a2 = self.robot.links[1].a
        d1 = self.robot.links[0].d
        px, py, pz = target_pos
        r = sqrt(px * px + py * py)
        cos_q2 = (r * r - a1 * a1 - a2 * a2) / (2 * a1 * a2)
        if abs(cos_q2) > 1.0 + 1e-9:
            raise RuntimeError("SCARA: target out of reach.")
        cos_q2 = max(-1.0, min(1.0, cos_q2))
        q2a = atan2(+sqrt(max(0.0, 1 - cos_q2 * cos_q2)), cos_q2)
        q2b = atan2(-sqrt(max(0.0, 1 - cos_q2 * cos_q2)), cos_q2)
        sols = []
        for q2 in (q2a, q2b):
            q1 = atan2(py, px) - atan2(a2 * sin(q2), a1 + a2 * cos(q2))
            q3 = d1 - pz  # Niku-style sign choice; user DH may flip sign
            q = np.zeros(self.n)
            q[0], q[1], q[2] = q1, q2, q3
            sols.append(q)
        print("\n--- SCARA R-R-P analytic derivation ---")
        print(f"r = {r:.6f}, cos(theta2) = {cos_q2:.6f}")
        for i, q in enumerate(sols):
            print(f"Branch {i+1}: theta1 = {degrees(q[0]):.6f}°, theta2 = {degrees(q[1]):.6f}°, d3 = {q[2]:.6f} m")
        print("--------------------------------------\n")
        return sols
    def analytic_cylindrical_prp(self, target_pos):
        if self.n < 3:
            raise RuntimeError("PRP requires at least 3 joints.")
        if not (self.robot.links[0].joint_type == 'P' and self.robot.links[1].joint_type == 'R' and self.robot.links[2].joint_type == 'P'):
            raise RuntimeError("Cylindrical PRP requires P-R-P pattern.")
        a1 = self.robot.links[0].a
        px, py, pz = target_pos
        q1 = pz
        q2 = atan2(py, px)
        r = sqrt(px * px + py * py)
        q3 = r - a1
        q = np.zeros(self.n)
        q[0], q[1], q[2] = q1, q2, q3
        print("\n--- Cylindrical PRP analytic derivation ---")
        print(f"q1 = {q1:.6f} m, theta2 = {degrees(q2):.6f}°, q3 = {q3:.6f} m")
        print("--------------------------------------\n")
        return [q]
    def analytic_2r_planar(self, target_pos):
        if self.n < 2:
            raise RuntimeError("2R requires at least 2 joints.")
        if not self._is_planar_first_n(2):
            raise RuntimeError("2R planar geometric condition not met.")
        a1 = self.robot.links[0].a
        a2 = self.robot.links[1].a
        px, py, pz = target_pos
        r = sqrt(px * px + py * py)
        if r < 1e-12:
            raise RuntimeError("Target at origin; infinite solutions.")
        cos_q2 = (r * r - a1 * a1 - a2 * a2) / (2 * a1 * a2)
        if abs(cos_q2) > 1.0 + 1e-9:
            raise RuntimeError("2R: target out of reach.")
        cos_q2 = max(-1.0, min(1.0, cos_q2))
        q2a = atan2(+sqrt(max(0.0, 1 - cos_q2 * cos_q2)), cos_q2)
        q2b = atan2(-sqrt(max(0.0, 1 - cos_q2 * cos_q2)), cos_q2)
        sols = []
        for q2 in (q2a, q2b):
            q1 = atan2(py, px) - atan2(a2 * sin(q2), a1 + a2 * cos(q2))
            q = np.zeros(self.n)
            q[0], q[1] = q1, q2
            sols.append(q)
        print("\n--- Planar 2R analytic derivation ---")
        for i, q in enumerate(sols):
            print(f"Branch {i+1}: theta1 = {degrees(q[0]):.6f}°, theta2 = {degrees(q[1]):.6f}°")
        print("--------------------------------------\n")
        return sols
    def analytic_3r_planar(self, target_pos):
        if self.n < 3:
            raise RuntimeError("3R requires at least 3 joints.")
        if not self._is_planar_first_n(3):
            raise RuntimeError("3R planar geometric condition not met.")
        d1 = self.robot.links[0].d
        a2 = self.robot.links[1].a
        a3 = self.robot.links[2].a
        px, py, pz = target_pos
        r = sqrt(px * px + py * py)
        s = pz - d1
        D = (r * r + s * s - a2 * a2 - a3 * a3) / (2 * a2 * a3)
        if abs(D) > 1.0 + 1e-9:
            raise RuntimeError("3R: target out of reach.")
        D = max(-1.0, min(1.0, D))
        theta3a = atan2(+sqrt(max(0.0, 1 - D * D)), D)
        theta3b = atan2(-sqrt(max(0.0, 1 - D * D)), D)
        sols = []
        for theta3 in (theta3a, theta3b):
            theta2 = atan2(s, r) - atan2(a3 * sin(theta3), a2 + a3 * cos(theta3))
            theta1 = atan2(py, px)
            q = np.zeros(self.n)
            q[0], q[1], q[2] = theta1, theta2, theta3
            sols.append(q)
        print("\n--- Planar 3R analytic derivation (Niku) ---")
        print(f"r = {r:.6f}, s = {s:.6f}, D = {D:.6f}")
        for i, q in enumerate(sols):
            print(f"Branch {i+1}: theta1 = {degrees(q[0]):.6f}°, theta2 = {degrees(q[1]):.6f}°, theta3 = {degrees(q[2]):.6f}°")
        print("--------------------------------------\n")
        return sols
    def analytic_6d_spherical_wrist(self, target_pos, target_ori):
        if self.n < 6:
            raise RuntimeError("6-DOF spherical wrist requires at least 6 joints.")
        if not self._is_spherical_wrist():
            raise RuntimeError("6-DOF spherical wrist geometry not detected.")
        roll, pitch, yaw = map(radians, target_ori)
        R_des = rpy_to_rot(roll, pitch, yaw)
        d6 = self.robot.links[5].d if self.n >= 6 else 0.0
        p_des = np.asarray(target_pos).reshape(3)
        z_des = R_des[:, 2]
        p_wc = p_des - d6 * z_des
        if not self._is_planar_first_n(3):
            raise RuntimeError("Spherical-wrist decoupling here requires first 3 joints planar (as implemented).")
        wrist_center_solutions = self.analytic_3r_planar(p_wc)
        full_solutions = []
        for q_first3 in wrist_center_solutions:
            q_temp = np.zeros(self.n)
            q_temp[:3] = q_first3[:3]
            T03 = self.robot.fk(q_temp)
            R03 = T03[:3, :3]
            R36 = R03.T @ R_des
            wrist_r, wrist_p, wrist_y = rot_matrix_to_rpy(R36)
            q = np.zeros(self.n)
            q[:3] = q_first3[:3]
            q[3], q[4], q[5] = wrist_r, wrist_p, wrist_y
            full_solutions.append(q)
        print("\n--- 6-DOF spherical wrist analytic (decoupling) ---")
        print(f"p_des = {p_des}, p_wc = {p_wc}, assumed d6 = {d6}")
        for i, q in enumerate(full_solutions):
            print(f"Branch {i+1}: first3 (deg) = {[degrees(v) for v in q[:3]]}, wrist (deg) = {[degrees(v) for v in q[3:6]]}")
        print("--------------------------------------\n")
        return full_solutions
    def solve(self, target_pos, target_ori=None, init_guess=None):
        pattern = self.joint_pattern()
        analytic_solutions = []
        # canonical matches (ordered)
        try:
            if pattern.startswith('PPP'):
                analytic_solutions.extend(self.analytic_cartesian_ppp(target_pos))
        except Exception as e:
            logging.info(f"PPP not applicable: {e}")
        try:
            if pattern.startswith('RRP'):
                analytic_solutions.extend(self.analytic_scara_rrp(target_pos))
        except Exception as e:
            logging.info(f"SCARA RRP not applicable: {e}")
        try:
            if pattern.startswith('PRP'):
                analytic_solutions.extend(self.analytic_cylindrical_prp(target_pos))
        except Exception as e:
            logging.info(f"PRP not applicable: {e}")
        try:
            if self._is_planar_first_n(2):
                analytic_solutions.extend(self.analytic_2r_planar(target_pos))
        except Exception as e:
            logging.info(f"2R planar not applicable: {e}")
        try:
            if self._is_planar_first_n(3):
                analytic_solutions.extend(self.analytic_3r_planar(target_pos))
        except Exception as e:
            logging.info(f"3R planar not applicable: {e}")
        try:
            if self.n >= 6 and target_ori is not None and self._is_spherical_wrist():
                analytic_solutions.extend(self.analytic_6d_spherical_wrist(target_pos, target_ori))
        except Exception as e:
            logging.info(f"6R spherical wrist not applicable: {e}")
        if analytic_solutions:
            print("\n=== Analytic solutions (textbook) ===")
            best = None
            best_err = 1e9
            for idx, q in enumerate(analytic_solutions):
                T = self.robot.fk(q)
                pos_err = np.linalg.norm(target_pos - T[:3, 3])
                print(f"\nBranch {idx+1}:")
                for i in range(self.n):
                    if self.robot.links[i].joint_type == 'R':
                        print(f"  q[{i}] = {q[i]:.6f} rad, {degrees(q[i]):.3f} deg")
                    else:
                        print(f"  q[{i}] = {q[i]:.6f} m (prismatic)")
                print(f"  Position error = {pos_err:.6e} m")
                if pos_err < best_err:
                    best_err = pos_err
                    best = q
            print("\nReturning best analytic branch (lowest position error).")
            return best
        raise RuntimeError("No textbook analytic solution available for this manipulator configuration and target. "
                           "As requested, numeric fallback has been removed. Try entering a manipulator that matches "
                           "a textbook analytic form (PPP, PRP, RRP/SCARA, planar 2R/3R, 6R spherical wrist with planar first 3), "
                           "or re-enable numeric IK in a different script.")
    #kranthi
# -------------------------
# URDF Download / Fetch logic
# -------------------------
def fetch_prebuilt_urdf(dof):
    """
    For dof in {2, 3, 6}, check local folder for a matching URDF file.
    If missing, download from GitHub raw URL; return local path.
    If dof is 1 or 5 (or unsupported), return None (so generate via DH).
    """
    urdf_map = {
        2: ("simple2d_arm.urdf",
            "https://raw.githubusercontent.com/akinami3/PybulletRobotics/main/urdf/simple2d_arm.urdf"),
        3: ("spatial_3R_robot.urdf",
            "https://raw.githubusercontent.com/adityasagi/robotics_tutorial/master/spatial_3R_robot.urdf"),
        6: ("simple6d_arm_with_gripper.urdf",
            "https://raw.githubusercontent.com/akinami3/PybulletRobotics/main/urdf/simple6d_arm_with_gripper.urdf"),
    }
    if dof not in urdf_map:
        return None
    local_fname, url = urdf_map[dof]
    os.makedirs("prebuilt_urdfs", exist_ok=True)
    local_path = os.path.join("prebuilt_urdfs", local_fname)
    if not os.path.exists(local_path):
        logging.info(f"Downloading URDF for DOF = {dof} from: {url}")
        try:
            urllib.request.urlretrieve(url, local_path)
            logging.info(f"Saved URDF to {local_path}")
        except Exception as e:
            logging.error(f"Failed to download URDF from {url}: {e}")
            return None
    else:
        logging.info(f"Using existing URDF file at {local_path}")
    return local_path
# -------------------------
# IO helpers (unchanged)
# -------------------------
def get_float(prompt, default=None):
    while True:
        val = input(prompt)
        if val.strip() == "" and default is not None:
            return default
        try:
            return float(val)
        except ValueError:
            print("Invalid number. Try again.")
def plot_joint_trajectory(csv_path='trajectory.csv'):
    if not os.path.exists(csv_path):
        print(f"No trajectory file at {csv_path}")
        return
    times = []
    joints = []
    with open(csv_path, 'r') as cf:
        reader = csv.reader(cf)
        header = next(reader, None)
        for row in reader:
            times.append(int(row[0]))
            joints.append([float(x) for x in row[1:]])
    joints = np.array(joints)
    plt.figure(figsize=(10, 6))
    for j in range(joints.shape[1]):
        plt.plot(times, joints[:, j], label=f'joint {j}')
    plt.xlabel('time step'); plt.ylabel('joint value (rad or m)')
    plt.legend(); plt.grid(True); plt.show()
# -------------------------
# Main interactive routine (keeps your original flow)
# -------------------------
def main():
    print("Graduate-Level DH Robotics Simulator with IK (Niku-textbook-only analytic methods)")
    n_links = int(get_float("Enter number of links (1-6): "))
    if n_links < 1 or n_links > 6:
        print("Invalid number of links (1-6).")
        return
    # Check for prebuilt URDF for this DOF (download if missing)
    prebuilt_path = fetch_prebuilt_urdf(n_links)
    use_prebuilt = (prebuilt_path is not None)

    links = []
    default_colors = [(0.3, 0.6, 0.9, 1), (0.9, 0.3, 0.3, 1), (0.3, 0.9, 0.3, 1),
                      (0.9, 0.9, 0.3, 1), (0.6, 0.3, 0.8, 1), (0.3, 0.4, 0.7, 1)]

    print("\nNOTE: For revolute joint limits enter degrees (e.g. -180, 180). For prismatic, enter meters (e.g. -0.5, 0.5).")
    for i in range(n_links):
        print(f"\nEnter parameters for link {i + 1}:")
        a = get_float("  Enter link length 'a' (m): ")
        alpha_deg = get_float("  Enter link twist 'alpha' (deg): ")
        d = get_float("  Enter link offset 'd' (m): ")
        theta_deg = get_float("  Enter initial joint angle 'theta' (deg): ")
        jt = input("  Enter joint type (R = revolute, P = prismatic): ").strip().upper()
        if jt not in ('R', 'P'):
            print("Invalid joint type.")
            return
        q_min_val = get_float("  Enter joint minimum limit (deg for R, m for P): ")
        q_max_val = get_float("  Enter joint maximum limit (deg for R, m for P): ")
        if jt == 'R':
            q_min = radians(q_min_val)
            q_max = radians(q_max_val)
        else:
            q_min = float(q_min_val)
            q_max = float(q_max_val)

        links.append(Link(a=a, alpha=radians(alpha_deg), d=d, theta=radians(theta_deg),
                           joint_type=jt, q_min=q_min, q_max=q_max, color=default_colors[i % len(default_colors)]))
    # DH table & Ai & T0_n (Niku-style)
    dh_rows = []
    Ai_matrices = []
    for idx, lk in enumerate(links, start=1):
        a_i_1 = lk.a
        alpha_i_1_deg = degrees(lk.alpha)
        d_i = lk.d
        theta_i_deg = degrees(lk.theta)
        dh_rows.append({
            "Link": idx,
            "a(i-1) (m)": f"{a_i_1:.6f}",
            "alpha(i-1) (deg)": f"{alpha_i_1_deg:.6f}",
            "d(i) (m)": f"{d_i:.6f}",
            "theta(i) (deg)": f"{theta_i_deg:.6f}"
        })
        Ai = dh_transform(lk.a, lk.alpha, lk.d, lk.theta)
        Ai_matrices.append(Ai)
    dh_df = pd.DataFrame(dh_rows, columns=["Link", "a(i-1) (m)", "alpha(i-1) (deg)", "d(i) (m)", "theta(i) (deg)"])
    print("\n--- Denavit-Hartenberg Table (Niku format) ---")
    print(dh_df.to_string(index=False))
    print("---------------------------------------------\n")
    print("--- Homogeneous transform A_i matrices (Ai) ---")
    for i, A in enumerate(Ai_matrices, start=1):
        print(f"\nA_{i} =")
        with np.printoptions(precision=6, suppress=True):
            print(A)
    T = np.eye(4)
    for A in Ai_matrices:
        T = T @ A
    print("\n--- Final homogeneous transform T_0_n ---")
    with np.printoptions(precision=6, suppress=True):
        print(T)
    print("--------------------------------------------------------------\n")
    save_dh = input("Save this DH table as CSV? (y/n): ").strip().lower() == 'y'
    if save_dh:
        dh_path = "DH_Table.csv"
        try:
            dh_df.to_csv(dh_path, index=False)
            print(f"DH table saved to: {dh_path}")
        except Exception as e:
            print(f"Failed to save DH table: {e}")
    coord_sys = input("Select target coordinate system (cartesian/cylindrical/spherical): ").strip().lower()
    if coord_sys not in ('cartesian', 'cylindrical', 'spherical'):
        print("Invalid coordinate system.")
        return
    if coord_sys == 'cartesian':
        tx = get_float("  Enter target X (m): ")
        ty = get_float("  Enter target Y (m): ")
        tz = get_float("  Enter target Z (m): ")
        target_pos = np.array([tx, ty, tz])
    elif coord_sys == 'cylindrical':
        r = get_float("  Enter radial distance r: ")
        th_deg = get_float("  Enter angle theta (deg): ")
        z = get_float("  Enter height z (m): ")
        th = radians(th_deg)
        target_pos = np.array([r * cos(th), r * sin(th), z])
    else:
        rho = get_float("  Enter radial distance rho: ")
        az_deg = get_float("  Enter azimuth angle theta (deg): ")
        el_deg = get_float("  Enter elevation angle phi (deg): ")
        az = radians(az_deg); el = radians(el_deg)
        target_pos = np.array([rho * sin(el) * cos(az), rho * sin(el) * sin(az), rho * cos(el)])
    incl_ori = input("Include end-effector orientation? (y/n): ").strip().lower() == 'y'
    target_ori = None
    if incl_ori:
        roll = get_float("  Enter roll angle (deg): "); pitch = get_float("  Enter pitch angle (deg): "); yaw = get_float("  Enter yaw angle (deg): ")
        target_ori = [roll, pitch, yaw]
    use_guess = input("Provide initial guess? (y/n): ").strip().lower() == 'y'
    q_init = None
    if use_guess:
        q_init_vals = []
        for i in range(n_links):
            prompt = f"  Initial guess for joint {i + 1} ({'deg' if links[i].joint_type == 'R' else 'm'}): "
            val = get_float(prompt)
            q_init_vals.append(radians(val) if links[i].joint_type == 'R' else float(val))
        q_init = np.array(q_init_vals)
    save_traj_csv = input("Save trajectory CSV? (y/n): ").strip().lower() == 'y'
    traj_csv_path = 'trajectory.csv' if save_traj_csv else None
    save_jointmap = input("Save joint mapping file? (y/n): ").strip().lower() == 'y'
    show_plot_after = input("Show joint trajectory plot after sim? (y/n): ").strip().lower() == 'y'
    robot = Robot(links)
    ik = IKSolver(robot)
    try:
        q_solution = ik.solve(target_pos, target_ori, init_guess=q_init)
        print("\nIK Solution (rad/deg or m):")
        for i, val in enumerate(q_solution):
            if links[i].joint_type == 'R':
                print(f"  Joint {i+1}: {val:.6f} rad, {degrees(val):.3f} deg")
            else:
                print(f"  Joint {i+1}: {val:.6f} m (prismatic)")
        robot.plot(q_solution)
        robot.run_sim(q_solution, use_prebuilt=use_prebuilt, prebuilt_urdf_path=prebuilt_path,
                      save_jointmap=save_jointmap, save_traj_csv=save_traj_csv,
                      traj_csv_path=traj_csv_path if traj_csv_path else 'trajectory.csv')
        downloads_dir = r"C:\Users\murik\OneDrive\Documents\python"
        if save_traj_csv:
            copy_traj = input(f"Do you want a copy of '{traj_csv_path}' saved to {downloads_dir}? (y/n): ").strip().lower() == 'y'
            if copy_traj:
                try:
                    os.makedirs(downloads_dir, exist_ok=True)
                    shutil.copy(traj_csv_path, os.path.join(downloads_dir, os.path.basename(traj_csv_path)))
                    print(f"Trajectory CSV copied to {os.path.join(downloads_dir, os.path.basename(traj_csv_path))}")
                except Exception as e:
                    print(f"Failed to copy trajectory CSV: {e}")
        if save_jointmap:
            if os.path.exists('joint_map.txt'):
                copy_jmap = input(f"Do you want a copy of 'joint_map.txt' saved to {downloads_dir}? (y/n): ").strip().lower() == 'y'
                if copy_jmap:
                    try:
                        os.makedirs(downloads_dir, exist_ok=True)
                        shutil.copy('joint_map.txt', os.path.join(downloads_dir, 'joint_map.txt'))
                        print(f"Joint map copied to {os.path.join(downloads_dir, 'joint_map.txt')}")
                    except Exception as e:
                        print(f"Failed to copy joint_map.txt: {e}")
            else:
                print("No joint_map.txt found to copy.")
        if save_traj_csv and show_plot_after:
            plot_joint_trajectory(traj_csv_path)
    except Exception as e:
        print(f"Error: {e}")
if __name__ == "__main__":
    main()