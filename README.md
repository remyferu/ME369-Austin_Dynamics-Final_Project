# ME369-Austin_Dynamics-Final_Project

#### Description:
This final project uses rospy and a custom A* algorithm to navigate a turtlebot through a manhattan grid with randomly changing obstacles in gazebo. 


####  SETUP/HOW TO: 

1. Please follow along with the instructions from the [lightning talk code follow along](https://docs.google.com/document/d/1zF7FCS2k8OG_VFYPCMd8QHJgtybkvfCLmpAyR_oQRPE/edit?usp=sharing). Before clicking the link, please keep in mind the name of the catkin workspace you are creating. Instead of creating a lightning_talk workspace, create a final_project_wc workspace. 

2. Now create a package inside the workspace src folder for the model_mover.py file. Reference the [creating a package tutorial](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) for help. The dependencies for this package will be `rospy`, `gazebo_msgs`, and `std_msgs`.

3. Set the `model_mover.py` files as an executable using `chmod +x file_path`. 

4. Rebuild the workspace by using `catkin_make` in your base workspace directory. 

5. Move the world and launch files into their respective directories: `~/ME369P/final_project_wc/src/turtlebot3_simulations/turtlebot3_gazebo/worlds` and  `~/ME369P/final_project_wc/src/turtlebot3_simulations/turtlebot3_gazebo/launch`

6. You are now ready to launch your gazebo world and run your model_mover.py node. Use `roslaunch turtlebot3_simulations turtlebot3_final_project.launch` in one shell and `rosrun final_project model_mover.py` in another. 

If there are issues finding `TURTLEBOT3_MODEL` or other errors ensure that you run `source devel/setup.bash` in your base directory, run `export TURTLEBOT3_MODEL=waffle_pi`, and run the ros setup.bash file. It is very helpfull to add these console commands to the .bashrc file to avoid any tediously repetative tasks. 
