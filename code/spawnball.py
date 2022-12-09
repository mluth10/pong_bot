
def spawn_object(x=0, y=0, z=0.1):
    import sys
    import copy
    import rospy
    import moveit_commander
    import moveit_msgs.msg
    import geometry_msgs.msg
    from gazebo_msgs.srv import SpawnModel
    from geometry_msgs.msg import Point, Pose, Quaternion
    # import ROS_MASTER_URI=http://master:11311
    initial_pose = Pose()
    initial_pose.position.x = 1
    initial_pose.position.y = 1
    initial_pose.position.z = 1

    # f = open('/home/kal87/model_editor_models/pingpong_ball/model.sdf','r')
    # sdff = f.read()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

    # spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    # spawn_model_prox("model", sdff, "robotos_name_space", initial_pose, "world")

    client = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    path = "/home/ajm133/catkin_ws/src/ajm133_rosintro/pingpong_ball/model.sdf"
    client(
        model_name = "model",
        model_xml = open(path, "r").read(),
        robot_namespace = "/moveit_commander",
        initial_pose = Pose(position= Point(0,0,2),orientation=Quaternion(0,0,0,0)),
        reference_frame = "world")
    
def main():
    spawn_object()
    
if __name__ == "__main__":
    main()
    

# spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
# spawn_model_client(
#     model_name='RoboCup SPL Ball',
#     model_xml=open('/home/kal87/model_editor_models/pingpong ball/model.sdf', 'r').read(),
#     robot_namespace='/foo',
#     initial_pose=Pose(position= Point(0,0,2),orientation=Quaternion(0,0,0,0)),"world"),
#     reference_frame='world'
# )

# https://answers.ros.org/question/337065/what-is-the-correct-way-to-spawn-a-model-to-gazebo-using-a-python-script/
# https://answers.gazebosim.org//question/5553/how-does-one-use-gazebospawn_sdf_model/

# import sys
# import copy
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# from gazebo_msgs.srv import SpawnModel
# from geometry_msgs.msg import Point, Pose, Quaternion
 
# try:
#     from math import pi, tau, dist, fabs, cos
# except:  # For Python 2 compatibility
#     from math import pi, fabs, cos, sqrt
 
#     tau = 2.0 * pi
 
#     def dist(p, q):
#         return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))
 
 
# from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list
 
# moveit_commander.roscpp_initialize(sys.argv)
# rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
 
# robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()
 
# group_name = "manipulator"
# move_group = moveit_commander.MoveGroupCommander(group_name)
 
# display_trajectory_publisher = rospy.Publisher(
#     "/move_group/display_planned_path",
#     moveit_msgs.msg.DisplayTrajectory,
#     queue_size=20,
# )
 
# def spawn_object(file_name, number, x, y, z=0.0254):
#     client = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
#     path = "/home/lmt53/catkin_ws/src/final_dependencies/src/chess_files_cws/" + file_name + ".sdf"
#     client(
#         model_name = file_name + str(number),
#         model_xml = open(path, "r").read(),
#         robot_namespace = "/moveit_commander",
#         initial_pose = Pose(position = Point(x, y, z),
#         orientation = Quaternion(0, 0, 0, 0)),
#         reference_frame = "world"
#     )