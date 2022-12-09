rosservice call /gazebo/set_model_configuration '{model_name: "final_assembly_backup", joint_names:['slider'], joint_positions:[0.02]}'

gz joint -m final_assembly_backup -j slider --pos-t 0.02

https://github.com/syuntoku14/fusion2urdf/blob/master/README.md

https://classic.gazebosim.org/tutorials?tut=ros_control

https://answers.ros.org/question/293260/controller-manager-how-to-correctly-launch-it/

https://answers.ros.org/question/233059/gazebo-set-joint-angles-by-ros/
