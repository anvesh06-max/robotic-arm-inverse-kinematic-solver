# robotic-arm-inverse-kinematic-solver
A Python-based robotic arm inverse kinematic solver implementing textbook analytic methods using Denavit–Hartenberg parameters. Includes forward kinematics, closed-form IK for standard manipulators (PPP, RRP, PRP, planar 2R/3R, 6R wrist), URDF generation, and PyBullet simulation visualization.
 Inverse Kinematic Solver

 Overview

This solver follows Saeed B. Niku’s robotics textbook conventions (2nd Edition) and provides only closed-form analytic IK solutions for standard manipulators.  
It supports multiple arm types, including **2R, 3R, SCARA (RRP), Cylindrical (PRP), Cartesian (PPP), and 6R spherical wrist robots.

It is designed for educational, research, and demonstration purposes — helping students and enthusiasts visualize kinematics in both matplotlib and PyBullet 3D simulation.



 Features

✅ Analytic-only IK (no numeric fallback)  
✅ Supports PPP, PRP, RRP (SCARA), 2R, 3R, and 6R spherical wrist robots  
✅ Generates DH table and transformation matrices automatically  
✅ Exports URDF model for simulation  
✅ Runs live animation in **PyBullet**  
✅ Plots trajectories and joint motion using Matplotlib  
✅ Saves joint mapping and trajectory as CSV  

 Project Structure

robotic-arm-inverse-kinematic-solver/
│
├── src/
│ └── niku_dh_pipeline.py # main Python program
│
├── data/
│ ├── DH_Table_2R.csv
│ ├── DH_Table_3R.csv
│ └── DH_Table_SCARA.csv
│
├── outputs/
│ ├── robot.urdf
│ ├── trajectory.csv
│ └── joint_map.txt
│
├── README.md
├── requirements.txt
├── LICENSE
└── .gitignore


 Example DH Inputs:

1.2R Planar Robot

Link,a(i-1) (m),alpha(i-1) (deg),d(i) (m),theta(i) (deg),joint_type,q_min,q_max
1,0.25,0.0,0.0,0.0,R,-180,180
2,0.25,0.0,0.0,0.0,R,-180,180

2.3R Planar Robot 

Link,a(i-1) (m),alpha(i-1) (deg),d(i) (m),theta(i) (deg),joint_type,q_min,q_max
1,0.25,0.0,0.0,0.0,R,-180,180
2,0.25,0.0,0.0,0.0,R,-180,180
3,0.1,0.0,0.0,0.0,R,-180,180
 
 3.SCARA (RRP) Robot (data/DH_Table_SCARA.csv)

Link,a(i-1) (m),alpha(i-1) (deg),d(i) (m),theta(i) (deg),joint_type,q_min,q_max
1,0.3,0.0,0.4,0.0,R,-180,180
2,0.2,0.0,0.0,0.0,R,-180,180
3,0.0,0.0,0.1,0.0,P,0.0,0.2


 Usage
Run the main script:


Enter the number of links (1–6) and DH parameters manually or load them from a CSV.

Choose coordinate system (Cartesian/Cylindrical/Spherical).

Provide the target position and (optionally) orientation.

The program will:

Compute Forward & Inverse Kinematics

Display 3D visualization (Matplotlib)

Export URDF and simulate in PyBullet

Save trajectory.csv and joint_map.txt

 Example Output
3D plot of robot links and frames (Matplotlib)

URDF generated and animated in PyBullet

Joint trajectory CSV plotted automatically

Enter number of links (1-6): 2

Enter parameters for link 1:
  a = 0.25
  alpha = 0
  d = 0
  theta = 0
  joint type = R
  q_min = -180
  q_max = 180

Enter parameters for link 2:
  a = 0.25
  alpha = 0
  d = 0
  theta = 0
  joint type = R
  q_min = -180
  q_max = 180

Select coordinate system (cartesian/cylindrical/spherical): cartesian
Enter target X (m): 0.3
Enter target Y (m): 0.1
Enter target Z (m): 0
Include end-effector orientation? (y/n): n
Output Example (Console):

lua
Copy code
--- Planar 2R analytic derivation ---
Branch 1: theta1 = 14.036°, theta2 = 52.564°
Branch 2: theta1 = 66.598°, theta2 = -52.564°
--------------------------------------

IK Solution (rad/deg):
Joint 1: 0.245 rad, 14.036 deg
Joint 2: 0.917 rad, 52.564 deg

URDF saved to robot.urdf
Trajectory saved to trajectory.csv
Joint map saved to joint_map.txt
Starting PyBullet simulation...
Generated Files:

Copy code
outputs/
 ├── robot.urdf
 ├── trajectory.csv
 └── joint_map.txt
A PyBullet window will open, showing the robotic arm moving to the target position.


Reference
Saeed B. Niku, Introduction to Robotics: Analysis, Control, Applications (2nd Ed.)

PyBullet Documentation: https://pybullet.org

NumPy, Matplotlib, and Pandas libraries
