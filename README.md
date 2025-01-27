# logbandpower
Neurorobotics Homework

# Come far funzionare tutto
1) Scarica il file e lascialo nella cartella Downloads, link:

https://drive.google.com/file/d/1uBr5xO4rIT2c4uyMv3Wp68hWRpg_plb_/view?usp=sharing

3) lancia questi comandi per creare una cartella con il worspace e tutto il resto

mkdir -p ros_ws/src
  
  cd ros_ws
  
  catkin_make
  
  cd src
  
  catkin_create_pkg logbandpower std_msgs rospy
  
  cd ..
  
  catkin_make 

  cd src
  
  git clone https://github.com/Fraf48/logbandpower.git
  
  cd ..
  
  catkin_make
  
  source devel/setup.bash

3) Lancia questo comando dentro la cartella /ros_ws 
  roscore

4) Avvia un altro terminale e lancia
  roslaunch logbandpower logbandpower.launch 






