Robotic Arm Inverse Kinematic Solver

A Python-based robotic arm inverse kinematic solver implementing textbook Denavit–Hartenberg (DH) conventions.
It computes forward and analytic inverse kinematics for standard manipulators such as 2R, 3R, SCARA, Cylindrical, Cartesian, and 6R spherical wrist robots.
The solver also supports URDF export and PyBullet simulation.

Overview

This project is a Python-based robotic arm inverse kinematic solver that follows textbook Denavit–Hartenberg (DH) conventions.
It focuses purely on analytic (closed-form) inverse kinematics, avoiding any numerical or approximate solutions.

Using this program, a user can ➜
• Define the DH parameters of a manipulator
• Compute both forward and inverse kinematics
• Automatically generate a URDF file
• Visualize the arm and motion in PyBullet
• Export joint trajectories to CSV

It’s designed to make robotic arm kinematics both visual and interactive.

⚙️ Supported Robot Types

The solver supports these standard manipulators analytically:

➜ Cartesian (PPP)
➜ Cylindrical (PRP)
➜ SCARA (RRP)
➜ Planar 2R and 3R arms
➜ 6R Spherical wrist robot

Each one uses only textbook equations — no iterative solvers.

🧩 Features

➜ Implements textbook-accurate DH transformations
➜ Analytic forward and inverse kinematics
➜ Automatic URDF generation
➜ Real-time visualization in PyBullet
➜ Matplotlib 3D plotting of robot frames
➜ CSV trajectory generation for motion study
➜ Joint limit handling for safer simulations

📂 Repository Layout

➜ src → contains the main solver script niku_dh_pipeline.py
➜ data → includes DH_Table_inputs.pdf with all manipulator DH parameters
➜ outputs → stores simulation and trajectory outputs
➜ Project_report.pdf → complete documentation of theory and results
➜ LICENSE → open MIT license for free use and modification
➜ README.md → project overview and usage guide
➜ requirements.txt → all Python dependencies

🧾 Example DH Table (SCARA RRP)
Link,a(i-1) (m),alpha(i-1) (deg),d(i) (m),theta(i) (deg),joint_type,q_min,q_max
1,0.3,0.0,0.4,0.0,R,-180,180
2,0.2,0.0,0.0,0.0,R,-180,180
3,0.0,0.0,0.1,0.0,P,0.1,0.3


This example defines a standard SCARA manipulator with two rotary joints and one prismatic joint.

🧮 How to Run

Open a terminal and go to the project folder
➜ cd src

Run the solver
➜ python niku_dh_pipeline.py

Follow the prompts:
• Enter link parameters (a, alpha, d, theta, joint type)
• Select the coordinate system (Cartesian / Cylindrical / Spherical)
• Enter the target position (and orientation if required)
• View results in the console, Matplotlib window, and PyBullet simulation

🧠 Background

The project is based on classic references:

• M. S. Niku, Introduction to Robotics: Analysis, Control, Applications (2nd Edition)
• Denavit & Hartenberg (1955), A Kinematic Notation for Lower-Pair Mechanisms
• PyBullet Documentation – for URDF-based simulation and visualization

📘 Input Reference

All manipulator DH parameters used in this project (2R, 3R, SCARA) are compiled in
data/DH_Table_inputs.pdf


🛠️ Installation

To install all required libraries:

pip install -r requirements.txt


Required Packages
• numpy
• matplotlib
• pandas
• pybullet

⚖️ License

This project is released under the MIT License.

You are free to use, modify, and distribute it with proper attribution.

💡 Acknowledgements

Special thanks to


💬 Final Note

This project bridges theory and practice in robotics.
By connecting Denavit–Hartenberg modeling, analytic inverse kinematics, and URDF-based simulation, it shows how mathematical models turn into real motion.

Robotics is not just about moving links – it’s about understanding how structure creates motion.”e consider starring the repository.
