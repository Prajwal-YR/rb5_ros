cd /root/rosws
roscore &
roslaunch rb5_vision rb_camera_main_ocv.launch
rosrun april_detection april_detection_node
rosrun april_detection mpi_twist_control_node.py
rosrun april_detection hw2.py