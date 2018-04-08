#Raspi=$1
File=$1
#ROS_MASTER_URI=http://raspi-$Raspi:11311
 rosrun dynamic_reconfigure dynparam load /reflekte $File
