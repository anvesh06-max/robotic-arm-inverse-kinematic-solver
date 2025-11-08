Robotic Arm Inverse Kinematic Solver

A Python-based robotic arm inverse kinematic solver implementing textbook Denavit–Hartenberg (DH) conventions.
It computes forward and analytic inverse kinematics for standard manipulators such as 2R, 3R, SCARA, Cylindrical, Cartesian, and 6R spherical wrist robots.
The solver also supports URDF export and PyBullet simulation.

Overview

This project follows the conventions from Saeed B. Niku’s Introduction to Robotics (2nd Edition) and provides closed-form (analytic) inverse kinematic solutions only, without using any numerical approximation.
It is developed for educational, research, and demonstration purposes, allowing users to visualize robotic kinematics through Matplotlib and PyBullet simulations.

Features

Analytic-only inverse kinematics (no numerical methods used)

Supports multiple manipulators:

2R and 3R planar arms

SCARA (RRP)

Cylindrical (PRP)

Cartesian (PPP)

6R spherical wrist

Automatic generation of DH tables and transformation matrices

URDF model export for simulation

3D visualization using Matplotlib

PyBullet simulation and animation support

Saves trajectory and joint mapping data to CSV files

Project Structure
robotic-arm-inverse-kinematic-solver/
│
├── src/
│   └── niku_dh_pipeline.py        # Main Python program
│
├── data/
│   ├── DH_Table_2R.csv
│   ├── DH_Table_3R.csv
│   └── DH_Table_SCARA.csv
│
├── outputs/
│   ├── robot.urdf
│   ├── trajectory.csv
│   └── joint_map.txt
│
├── README.md
├── requirements.txt
├── LICENSE
└── .gitignore

Example DH Parameter Files
2R Planar Robot
Link,a(i-1) (m),alpha(i-1) (deg),d(i) (m),theta(i) (deg),joint_type,q_min,q_max
1,0.25,0.0,0.0,0.0,R,-180,180
2,0.25,0.0,0.0,0.0,R,-180,180

3R Planar Robot
Link,a(i-1) (m),alpha(i-1) (deg),d(i) (m),theta(i) (deg),joint_type,q_min,q_max
1,0.25,0.0,0.0,0.0,R,-180,180
2,0.25,0.0,0.0,0.0,R,-180,180
3,0.1,0.0,0.0,0.0,R,-180,180

SCARA (RRP) Robot
Link,a(i-1) (m),alpha(i-1) (deg),d(i) (m),theta(i) (deg),joint_type,q_min,q_max
1,0.3,0.0,0.4,0.0,R,-180,180
2,0.2,0.0,0.0,0.0,R,-180,180
3,0.0,0.0,0.1,0.0,P,0.0,0.2

Usage Instructions

To run the solver, use:

python3 src/niku_dh_pipeline.py


Steps:

Enter the number of links (1–6).

Enter DH parameters manually or load them from a CSV file.

Choose the coordinate system (cartesian, cylindrical, or spherical).

Provide the target position and, optionally, the end-effector orientation.

The program will:

Compute forward and inverse kinematics.

Display a 3D robot visualization using Matplotlib.

Export a URDF and start a PyBullet simulation.

Save output data in the outputs folder.

Example Console Run
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

Select coordinate system: cartesian
Enter target X (m): 0.3
Enter target Y (m): 0.1
Enter target Z (m): 0
Include end-effector orientation? (y/n): n

Example Output
--- Planar 2R Analytic Derivation ---
Branch 1: theta1 = 14.036°, theta2 = 52.564°
Branch 2: theta1 = 66.598°, theta2 = -52.564°

IK Solution (rad/deg):
Joint 1: 0.245 rad, 14.036 deg
Joint 2: 0.917 rad, 52.564 deg

URDF saved to robot.urdf
Trajectory saved to trajectory.csv
Joint map saved to joint_map.txt
Starting PyBullet simulation...


Output files generated:

outputs/
 ├── robot.urdf
 ├── trajectory.csv
 └── joint_map.txt


When the process completes, a PyBullet window opens and visualizes the robotic arm moving to the specified target position.

Dependencies

To install all required Python libraries, run:

pip install -r requirements.txt


The requirements file should include:

numpy
pandas
matplotlib
pybullet

References

Saeed B. Niku, Introduction to Robotics: Analysis, Control, Applications (2nd Edition)

PyBullet Documentation: https://pybullet.org

NumPy: https://numpy.org

Matplotlib: https://matplotlib.org

Pandas: https://pandas.pydata.org

License

This project is licensed under the MIT License.
See the LICENSE file for details.

Author

Anvesh M
B.Tech in Automation and Robotics, Amrita University
Interested in robotics, kinematics, and simulation.

If you find this project useful, please consider starring the repository.
