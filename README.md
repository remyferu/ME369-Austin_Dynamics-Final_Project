# ME369-Austin_Dynamics-Final_Project

### Description:
This final project uses rospy and a custom A* algorithim to nagigate a turtlebot through a dynamic environemnt in gazebo. 


###  SETUP/HOW TO: 

-------------------------------------------------

please follow along with the instructions from this document (lightning talk code follow along):

Before clicking the link please keep in mind the name of the catking workspace you are creating. Instead of creating a lightning_talk workspace, create a final_project_wc workspace

https://docs.google.com/document/d/1zF7FCS2k8OG_VFYPCMd8QHJgtybkvfCLmpAyR_oQRPE/edit?usp=sharing

-------------------------------------------------

Now create a package inside the workspace src folder for the model_mover.py file. Reference the link below for help: 

http://wiki.ros.org/ROS/Tutorials/CreatingPackage

-------------------------------------------------

Tell linux that the model_mover.py is an executable: 

https://www.quora.com/How-do-you-make-a-Python-executable-in-Linux

-------------------------------------------------

Rebuild the workspace (catkin_make)

-------------------------------------------------

Place the world and launch files from the github into their respective folders 

~/ME369P/final_project_wc/src/turtlebot3_simulations/turtlebot3_gazebo/worlds 

~/ME369P/final_project_wc/src/turtlebot3_simulations/turtlebot3_gazebo/launch

-------------------------------------------------

You are now ready to launch your gazebo world and run your model_mover.py node

roslaunch turtlebot3_simulations turtlebot3_final_project.launch

rosrun final_project model_mover.py
