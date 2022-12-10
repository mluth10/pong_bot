# sources:
#https://manpages.ubuntu.com/manpages/bionic/man1/gz.1.html
# https://stackoverflow.com/questions/4256107/running-bash-commands-in-python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Point, Pose, Quaternion
from math import pi
import subprocess

# function for deleting balls so we can respawn them
def del_model(modelName):
    del_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    del_model_prox(modelName)

# function for spawning the table
def spawn_table(x=0, y=0, z=0.1):
    # import ROS_MASTER_URI=http://master:11311
    initial_pose = Pose()
    initial_pose.position.x = 1
    initial_pose.position.y = 1
    initial_pose.position.z = 1

    # spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnMod>
    # spawn_model_prox("model", sdff, "robotos_name_space", initial_pose, "world")

    client = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    path = "/home/kal87/model_editor_models/3rd pong/model.sdf"
    client(
        model_name="table",
        model_xml=open(path, "r").read(),
        robot_namespace="/moveit_commander",
        initial_pose=Pose(position=Point(x, y, z), orientation=Quaternion(0, 0, 0, 0)), reference_frame="world")

# function for spawning the ball
def spawn_ball(x=0, y=0, z=0.1):
    # import ROS_MASTER_URI=http://master:11311
    initial_pose = Pose()
    initial_pose.position.x = 1
    initial_pose.position.y = 1
    initial_pose.position.z = 1

    # spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnMod>
    # spawn_model_prox("model", sdff, "robotos_name_space", initial_pose, "world")

    client = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    path = "/home/kal87/model_editor_models/pingpong ball/model.sdf"
    client(
        model_name="ball",
        model_xml=open(path, "r").read(),
        robot_namespace="/moveit_commander",
        initial_pose=Pose(position=Point(x, y, z), orientation=Quaternion(0, 0, 0, 0)), reference_frame="world")

#function for making the shots
def ten_shots():
    # spawn the table with the cups
    spawn_table(0, 0, .933)
    # move robot up
    rospy.sleep(1)
    bashCommand = "rosservice call /gazebo/set_model_state"
    commandList = bashCommand.split()
    commandList.append('{model_state: { model_name: final_assembly_backup, pose: { position: { x: 0.990483, y: 0.001722 ,z: 1.015834 }, orientation: {x: 0, y: 0, z: 0, w: 0 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }')
    process = subprocess.Popen(commandList, stdout=subprocess.PIPE)
    output, error = process.communicate()

    #define a list of joint states to accomplish
    j_goals = [[90, -42, 4], [92, -42, 4.1], [95, -42, 4],
               [88, -40, 4], [85, -42, 3], [94, -42, 4], [96, -42, 4], [90, -41, 3.9], [95, -42, 4], [89, -40, 4]]
    n = len(j_goals)
    for i in range(0, n):
        # reset cannon
        bashCommand = "gz joint -m final_assembly_backup -j pan --pos-t 0"
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        bashCommand = "gz joint -m final_assembly_backup -j tilt --pos-t 0"
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        process.communicate()
        bashCommand = "gz joint -m final_assembly_backup -j slider --pos-t -.04"
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()

        # spawn the pingpong ball above the cannon
        spawn_ball(0.990483, 0.001722, 1.2)
        rospy.sleep(2)

        # got to joint goals (in radians)
        pos = [j_goals[i][0] * pi / 180, j_goals[i][1] * pi / 180]
        pos_pan = str(pos[0])
        pos_tilt = str(pos[1])
        # pan
        bashCommand = "gz joint -m final_assembly_backup -j pan --pos-t " + pos_pan
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        # tilt
        bashCommand = "gz joint -m final_assembly_backup -j tilt --pos-t " + pos_tilt
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()

        # shoot the ball before going to the next joint goal
        force_slider = str(j_goals[i][2])
        bashCommand = "gz joint -m final_assembly_backup -j slider -f " + force_slider
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        # wait then delete ball
        rospy.sleep(5)
        del_model("ball")
        # reset the force to zero so the slider can move
        bashCommand = "gz joint -m final_assembly_backup -j slider -f 0"
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()


def main():
    try:
        print("Let's play pong!")
        input(
            "============ Press `Enter`============="
        )
        ten_shots()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

