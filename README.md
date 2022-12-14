# Pentagonal_plotting_robot_repo
This repo contains the Inverse Kinematics and Forward Kinematics of a 5 bar parallel Linkage robot with 2  degree of freedom which operates in a 2d plane. It is used with a magnetic board to plot various shapes. The robot takes a sequence of points as input and goes from 1 point to next in a straight line.

The file contains: 5_bar_parallel_linkage_writer_square.ino (an Arduino source file with the inverse and forward kinematics of the robot with relevant driver code.)
                   5_bar_parallel_linkage_writer_triangle.ino (Arduino code for plotting a triangle)
                   Schematics circuit diagram and other diagrams.
                   Images of my setup.
                   Video of the arm in action.
              
NOTE: one can change the arm dimensions in these codes by changing the arm object parameters. The shape can be changed by changing the sequence of points(initial point must be same as ending point for smooth transition.)    

