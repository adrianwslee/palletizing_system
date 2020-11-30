#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @author   Adrian Lee (wslee@potenit.com)
# @
import rospy
import os
import threading, time
import sys
import moveit_commander
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion
from math import atan2

from math import radians, degrees
from geometry_msgs.msg import Pose, PoseStamped
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Int16
import moveit_commander
import moveit_msgs.msg
import numpy as np

sys.dont_write_bytecode = True
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../doosan-robot/common/imp")))  # get import pass : DSR_ROBOT.py

# for single robot 
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m1013"
import DR_init

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

from DSR_ROBOT import *

class movegroup_python(object):
    def __init__(self):
        super(movegroup_python, self).__init__()
        self.pub_stop = rospy.Publisher('/' + ROBOT_ID + ROBOT_MODEL + '/stop', RobotStop, queue_size=10)
        self.sub_cmd = rospy.Subscriber("/cmd", Int16, self.cmd_cb)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        self.sub_realsense = rospy.Subscriber('/arm/jobPose', PoseStamped, self.sense_cb)

        self.planning_frame = self.group.get_planning_frame()
        self.eef_link = self.group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()

        self.pose_transformed = PoseStamped()

        # Misc variables
        self.cmd_load = 0
        self.callback_enabled = False
        self.valid_target = False
        self.check_load = False
        self.check_unload = False
        self.check_standby = False
        self.emergency = False
        self.node = 0
        self.impossible_path = False
        self.box_angle = 0

    def go_to_joint_state(self, joint_goal):
        traj = self.group.plan(joint_goal)
        self.group.execute(traj, wait=True)
        current_joints = self.group.get_current_joint_values()
        return self.all_close(joint_goal, current_joints, 0.001)

    def go_to_pose_goal(self, pose_goal):
        self.group.set_pose_target(pose_goal)
        pose_traj = self.group.plan(pose_goal)
        while len(pose_traj.joint_trajectory.points) > 20:
            pose_traj = self.group.plan(pose_goal)
            print('replanning')

        self.group.execute(pose_traj, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose
        return self.all_close(pose_goal, current_pose, 0.001)

    def plan_and_go_movej(self, pose_goal):
        self.group.set_pose_target(pose_goal)
        pose_traj = self.group.plan(pose_goal)
        while len(pose_traj.joint_trajectory.points) > 30:
            print(len(pose_traj.joint_trajectory.points))
            pose_traj = self.group.plan(pose_goal)
            print('replanning')
        '''
        #plan using moveit & execute movej 
        rospy.logerr(pose_traj.joint_trajectory.points)
        
        for i in range(0,len(pose_traj.joint_trajectory.points)):
          pose_traj.joint_trajectory.points[i].positions = pose_traj.joint_trajectory.points[i].positions * 180/3.14
          print("pose : %d" %pose_traj[i])
        '''

    def go_movesj(self, pose_goal, rotate=False):
        # Quaternion Message is not compatible with tf quaternion type.
        if rotate is True:
            quat_orig = (pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w)
            quat_rot = quaternion_from_euler(0, 0, pi * 0.5)
            quat_new = quaternion_multiply(quat_rot, quat_orig)
            pose_goal.orientation.x = quat_new[0]
            pose_goal.orientation.y = quat_new[1]
            pose_goal.orientation.z = quat_new[2]
            pose_goal.orientation.w = quat_new[3]
        self.group.set_pose_target(pose_goal)
        pose_traj = self.group.plan(pose_goal)

        if len(pose_traj.joint_trajectory.points) == 0:
            return 2
        trajectory_options = []

        for options in range(10):
            trajectory_options.append(self.group.plan(pose_goal))

        pose_traj = min(trajectory_options, key=lambda traj: len(traj.joint_trajectory.points))

        trajectory_list = []
        improved_path_list = []
        for i in range(0, len(pose_traj.joint_trajectory.points)):
            position_list = []
            for pose in range(len(pose_traj.joint_trajectory.points[i].positions)):
                position_list.append(degrees(pose_traj.joint_trajectory.points[i].positions[pose]))
            trajectory_list.append(posj(position_list))

        trajectories_number = len(trajectory_list)
        improved_path_list.append(trajectory_list[0])
        improved_path_list.append(trajectory_list[int(trajectories_number / 2)])
        # improved_path_list.append(trajectory_list[int(trajectories_number / 3) * 2])
        improved_path_list.append(trajectory_list[trajectories_number - 1])


        # for path in range(0, len(trajectory_list)):
        #     movej(trajectory_list[path], vel = 75, acc = 75)

        # movesj(improved_path_list, vel=200, acc=200)
        movej(improved_path_list[2], vel=150, acc=150)
        current_pose = self.group.get_current_pose().pose
        return self.all_close(pose_goal, current_pose, 0.005)

    def simplified_movesj(self, pose_goal):
        self.group.set_pose_target(pose_goal)
        pose_traj = self.group.plan(pose_goal)

        if len(pose_traj.joint_trajectory.points) == 0:
            return 2
        while len(pose_traj.joint_trajectory.points) > 20:
            pose_traj = self.group.plan(pose_goal)
            print('replanning')

        trajectory_list = []
        improved_path_list = []
        for i in range(0, len(pose_traj.joint_trajectory.points)):
            position_list = []
            for pose in range(len(pose_traj.joint_trajectory.points[i].positions)):
                position_list.append(degrees(pose_traj.joint_trajectory.points[i].positions[pose]))
            trajectory_list.append(posj(position_list))

        trajectories_number = len(trajectory_list)
        improved_path_list.append(trajectory_list[0])
        # improved_path_list.append(trajectory_list[int(trajectories_number / 2)])
        improved_path_list.append(trajectory_list[trajectories_number - 1])

        # movesj(improved_path_list, vel=75, acc=75)
        movej(improved_path_list[1], vel=150, acc=150)
        current_pose = self.group.get_current_pose().pose
        return self.all_close(pose_goal, current_pose, 0.005)

    def plan_cartesian_path(self, scale=1):
        waypoints = []
        wpose = self.group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        # wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        # waypoints.append(copy.deepcopy(wpose))
        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        return plan, fraction

    def execute_plan(self, plan):
        self.group.execute(plan, wait=True)

    def current_pose(self):
        current_pose = self.group.get_current_pose().pose
        return current_pose

    def current_joint(self):
        current_joints = self.group.get_current_joint_values()
        return current_joints

    def current_angle(self):
        current_joints = self.group.get_current_joint_values()
        current_joints = [degrees(rads) for rads in current_joints]
        return posj(current_joints[0], current_joints[1], current_joints[2], current_joints[3], current_joints[4], current_joints[5])

    def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in self.scene.get_known_object_names()
            test = self.scene.get_known_object_names()
            print(test)

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def add_box(self, box_name, pos_x, pos_y, pos_z, size_x, size_y, size_z, timeout=4):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.pose.position.x = pos_x
        box_pose.pose.position.y = pos_y
        box_pose.pose.position.z = pos_z
        box_pose.header.frame_id = "/world"
        box_pose.pose.orientation.w = 1.0
        time.sleep(1)
        self.scene.add_box(box_name, box_pose, size=(size_x, size_y, size_z))
        return self.wait_for_state_update(box_name, box_is_known=True, timeout=timeout)

    def attach_box(self, box_name, timeout=4):
        grasping_group = 'arm'
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, box_name, touch_links=touch_links)
        return self.wait_for_state_update(box_name, box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, box_name, timeout=4):
        self.scene.remove_attached_object(self.eef_link, name=box_name)
        return self.wait_for_state_update(box_name, box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, box_name, timeout=4):
        self.scene.remove_world_object(box_name)
        return self.wait_for_state_update(box_name, box_is_attached=False, box_is_known=False, timeout=timeout)

    def remove_objects(self, remove_all=False, object_type=None):
        objects_list = self.scene.get_known_object_names()
        if remove_all is True:
            for obj in objects_list:
                self.scene.remove_world_object(obj)
            return True
        elif remove_all is False and object_type is not None:
            filtered_objects = [item for item in objects_list if object_type in item]
            print(filtered_objects)
            for obj in filtered_objects:
                self.scene.remove_world_object(obj)
            return True
        else:
            rospy.logdebug("No objects to remove")
            return False

    def all_close(self, goal, actual, tolerance):
        all_equal = True
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    rospy.logerr("Your manipulator hasn't reached to the goal yet")
                    return 1
                else:
                    rospy.loginfo("Your manipulator has reached to the goal pose")
                    return 0

        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is geometry_msgs.msg.Pose:
            return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    def shutdown(self):
        print("Shutting down")
        self.pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
        return 0

    def cmd_cb(self, msg):
        if self.cmd_load != msg.data:
            self.cmd_load = msg.data
            self.emergency = False
            if msg.data == 4:
                self.emergency = True
        self.callback_enabled = True

    def node_cb(self, msg):
        self.node = msg.data

    def sense_cb(self, msg):
        camera_pose = msg
        yaw_coord = PoseStamped()
        yaw_coord.pose.position.x = msg.pose.orientation.x
        yaw_coord.pose.position.y = msg.pose.orientation.y
        yaw_coord.pose.position.z = msg.pose.orientation.z

        camera_pose.header.stamp = rospy.Time.now()
        if msg.pose.position.x == 0 and msg.pose.position.y == 0 and msg.pose.position.z == 0:
            self.valid_target = False
        else:
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer)
            try:
                tf_transformer = tf_buffer.lookup_transform("base_0", "camera_link", rospy.Time(),
                                                            rospy.Duration(3.0))  # camera --> object coordinate
                self.pose_transformed = tf2_geometry_msgs.do_transform_pose(camera_pose, tf_transformer)
                pose_transformed2 = tf2_geometry_msgs.do_transform_pose(yaw_coord, tf_transformer)
                self.box_angle = np.rad2deg(atan2(yaw_coord.pose.position.y-self.pose_transformed.pose.position.y, yaw_coord.pose.position.x-self.pose_transformed.pose.position.x))
                rospy.loginfo("senseCb called")
                self.valid_target = True
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                time.sleep(1)
                rospy.logerr("No source/target frame")
                self.valid_target = False


class PathPlanning(movegroup_python):
    set_robot_mode = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/system/set_robot_mode', SetRobotMode)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    set_velx(30, 20)  # set global task speed: 30(mm/sec), 20(deg/sec)
    set_accx(60, 40)  # set global task accel: 60(mm/sec2), 40(deg/sec2)

    def __init__(self):
        super(movegroup_python, self).__init__()
        movegroup_python.__init__(self)
        self.box_number = 1
        self.standby_pose = posj(90.0, 35.0, 100.5, 0.0, 45.0, 0.0)
        self.unload_safe = posj(0.0, 16.0, 90.0, 0.0, 74.0, 0.0)
        # self.node_number = [[0.66, 0.4], [1.0, 0.4], [0.66, 0.13], [1.0, 0.13], [0.66, -0.14], [1.0, -0.14]]
        self.node_number = [[1.02, -0.14], [0.66, -0.14], [1.02, 0.13], [0.66, 0.13], [1.02, 0.4], [0.66, 0.4]]
        self.level = 0
        self.pick_box = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/io/set_tool_digital_output',
                                      SetToolDigitalOutput)

    def go_down(self, in_m):
        going_down = self.current_pose()
        going_down.position.z = going_down.position.z - in_m
        return self.simplified_movesj(going_down)

    def go_up(self, in_m):
        going_up = self.current_pose()
        going_up.position.z = going_up.position.z + in_m
        return self.simplified_movesj(going_up)

    def compare_posj(self, desired_posj, tol):
        current_posj = self.current_joint()
        if len(current_posj) == len(desired_posj):
            for index in range(1, len(current_posj)):
                if abs(degrees(current_posj[index]) - desired_posj[index]) > tol:
                    return False
                else:
                    return True
                print("Index : %d" % index)

    def object_destination(self, direction):  # True: Right / False: Left //Currently Not used
        object_ = self.current_pose()
        if direction == 0:
            object_.position.x = object_.position.x  # Must Change X to Y / Where Cable is located is negative Y
        elif direction == 1:
            object_.position.x = object_.position.x - 0.2
        elif direction == 2:
            object_.position.x = object_.position.x + 0.2
        return object_

    def execute_and_check_path(self, goal_pose):
        if self.go_movesj(goal_pose) == 2:
            self.impossible_path = True
            raise done
        while self.go_movesj(goal_pose) == 1:
            pass

    def standby(self):
        print("2")
        pass

    def load(self, pose_transformed):
        load_pose = self.current_pose()
        # roll, pitch, yaw = euler_from_quaternion([load_pose.orientation.x, load_pose.orientation.y, load_pose.orientation.z, load_pose.orientation.w])
        # print(np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw))

        load_pose.position.x = pose_transformed.pose.position.x + 0.10
        load_pose.position.y = pose_transformed.pose.position.y - 0.1
        load_pose.position.z = pose_transformed.pose.position.z + 0.572

        rotate_test = self.current_angle()
        rotate_test[5] = rotate_test[5] + self.box_angle - 80
        print(self.box_angle)
        movej(rotate_test, vel=50, acc=50)
        rospy.logerr(load_pose)
        after_math = self.current_pose()
        load_pose.orientation = after_math.orientation
        # self.add_box("box" + str(self.box_number), box_pos_x, box_pos_y, box_pos_z-0.3, 0.34, 0.25, 0.21)
        for retry in range(5):
            check_function = self.go_movesj(load_pose)
            while check_function == 1:
                self.go_movesj(load_pose)
            if check_function == 2:
                self.impossible_path = True
                rospy.logerr("Impossible Path")
                break
            self.impossible_path = False
            # self.attach_box("box" + str(self.box_number))
            break  # Exit retry
        # self.go_down(0.18)
        self.pick_box(1, 1)
        time.sleep(0.5)
        self.go_up(0.3)

    def unload(self, box_pos_x, box_pos_y, box_pos_z, node):
        unload_pose = self.current_pose()
        unload_pose.position.x = node[0]
        unload_pose.position.y = node[1]
        if self.level == 1:
            unload_pose.position.z = 0.3
        elif self.level == 2:
            unload_pose.position.z = 0.5
        for retry in range(5):
            check_function = self.go_movesj(unload_pose, rotate=True)
            while check_function == 1:
                self.go_movesj(unload_pose)
            if check_function == 2:
                self.impossible_path = True
                rospy.logerr("Impossible Path")
                break
            self.impossible_path = False
            break
        if self.level == 1:
            self.go_down(0.28)
        elif self.level == 2:
            self.go_down(0.26)
        self.pick_box(1, 0)
        # self.detach_box("box" + str(self.box_number))
        self.box_number = self.box_number + 1
        time.sleep(1)
        if self.level == 1:
            self.go_up(0.3)
        elif self.level == 2:
            self.go_up(0.3)

    def deconstruct(self):
        pass

    def main(self):
        rospy.init_node('palletizing_node', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        # Get Standby Pose
        movej(self.standby_pose, vel=20, acc=20)

        # self.remove_objects(remove_all=True)
        # for index, box in enumerate(self.node_number):
        #     self.add_box("box" + str(index), self.node_number[index][0], self.node_number[index][1], -0.5, 0.34, 0.25, 0.21)
        #
        self.remove_objects(remove_all=True)

        self.level = 1
        while not rospy.is_shutdown():
            if self.cmd_load == 1:
                for index, box in enumerate(self.node_number):
                    movej(self.standby_pose, vel=80, acc=80)
                    #self.load(-0.4, 1.1, -0.1)
                    # self.load(0, 1.0, 0)
                    self.load(self.pose_transformed)
                    movej(self.unload_safe, vel=80, acc=80)
                    self.unload(0.75, -0.06, -0.1, self.node_number[index])
                else:
                    print("Level " + str(self.level) + " completed")
                    self.level = self.level + 1
                if self.impossible_path is True:
                    movej(self.standby_pose, vel=100, acc=100)
            if self.level == 3:
                self.remove_objects(remove_all=True)
                print("All tasks completed")
                break # For Demo
            elif self.cmd_load == 10:
                self.pick_box(1, 0) # Shutdown Pump
                rospy.signal_shutdown("Shutdown Cmd Received")
        print('good bye!')


if __name__ == '__main__':
    path_planning = PathPlanning()
    path_planning.main()
