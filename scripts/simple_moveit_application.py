#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.srv import GetRobotStateFromWarehouse as GetState
import threading
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK
from actionlib import SimpleActionClient
from std_msgs.msg import Header


class Ur5Commander(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._robot_name = self._robot._r.get_robot_name()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander("manipulator")
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory)

        self._warehouse_name_get_srv = rospy.ServiceProxy("get_robot_state", GetState)

        self._joint_states_lock = threading.Lock()
        self._joint_states_listener = \
            rospy.Subscriber("joint_states", JointState,
                             self._joint_states_callback, queue_size=1)

        self._joints_position = {}
        self._joints_velocity = {}
        self._joints_effort = {}
        self.__plan = None

        self._forward_k = rospy.ServiceProxy(
            'compute_fk', GetPositionFK)
        self._forward_k = rospy.ServiceProxy(
            'compute_fk', GetPositionFK)

        self._set_up_action_client()

        # TODO(Vahid): check if this is necessary
        # rospy.sleep(10)

    def _joint_states_callback(self, joint_state):
        """
            The callback function for the topic joint_states.
            It will store the received joint position, velocity and efforts
            information into dictionaries
            @param joint_state - the message containing the joints data.
            """
        with self._joint_states_lock:
            self._joints_position = {n: p for n, p in
                                     zip(joint_state.name,
                                         joint_state.position)}
            self._joints_velocity = {n: v for n, v in
                                     zip(joint_state.name,
                                         joint_state.velocity)}
            self._joints_effort = {n: v for n, v in
                                   zip(joint_state.name, joint_state.effort)}

    def get_end_effector_pose_from_named_state(self, name):
        state = self._warehouse_name_get_srv(name, self._robot_name).state
        return self.get_end_effector_pose_from_state(state)

    def get_end_effector_pose_from_state(self, state):
        header = Header()
        fk_link_names = [self._move_group_commander.get_end_effector_link()]
        header.frame_id = self._move_group_commander.get_pose_reference_frame()
        response = self._forward_k(header, fk_link_names, state)
        return response.pose_stamped[0]

    def _set_up_action_client(self):
        """
        Sets up an action client to communicate with the trajectory controller
        """
        pass
        # self._action_running = False
        #
        # self._client = SimpleActionClient(
        #     self._prefix + "trajectory_controller/follow_joint_trajectory",
        #     FollowJointTrajectoryAction
        # )
        #
        # if self._client.wait_for_server(timeout=rospy.Duration(4)) is False:
        #     rospy.logfatal("Failed to connect to action server in 4 sec")
        #     raise

    def get_info(self):
        rospy.loginfo("Reference frame is " + self._group.get_planning_frame())
        if self._group.has_end_effector_link():
            rospy.loginfo("End effector name is " + self._group.get_end_effector_link())
        rospy.loginfo("List of robot groups ")
        rospy.loginfo(self._robot.get_group_names())
        rospy.loginfo("Robot state is")
        rospy.loginfo(self._robot.get_current_state())
        rospy.loginfo("robot name is " + self._robot_name)
        rospy.logwarn(self._group.get_current_pose(end_effector_link="ee_link"))

        # self._warehouse_name_get_srv.call()

        for _ in range(0, 1):
            self._group.clear_pose_targets()
            current_pose = self._group.get_current_pose(end_effector_link="ee_link")
            current_pose = current_pose.pose
            # rospy.logwarn(type(current_pose))
            current_pose.position.x += 0.05
            self._group.set_pose_target(current_pose)
            self._group.set_planning_time(10)

            plan1 = self._group.plan()
            # print "============ Waiting while RVIZ displays plan1..."
            self._group.go(wait=True)
            rospy.sleep(2)

            self._group.clear_pose_targets()

            current_pose = self._group.get_current_pose(end_effector_link="ee_link")
            current_pose = current_pose.pose
            # rospy.logwarn(type(current_pose))
            current_pose.position.z += 0.05
            self._group.set_pose_target(current_pose)
            plan1 = self._group.plan()
            # print "============ Waiting while RVIZ displays plan1..."
            self._group.go(wait=True)
            rospy.sleep(2)

            self._group.clear_pose_targets()

            current_pose = self._group.get_current_pose(end_effector_link="ee_link")
            current_pose = current_pose.pose
            # rospy.logwarn(type(current_pose))
            current_pose.position.x -= 0.05
            self._group.set_pose_target(current_pose)
            self._group.set_planning_time(3)
            plan1 = self._group.plan()
            # print "============ Waiting while RVIZ displays plan1..."
            self._group.go(wait=True)
            self._group.get_planning_time()
            rospy.sleep(2)

            self._group.clear_pose_targets()

            current_pose = self._group.get_current_pose(end_effector_link="ee_link")
            current_pose = current_pose.pose
            # rospy.logwarn(type(current_pose))
            current_pose.position.z -= 0.05
            self._group.set_pose_target(current_pose)
            plan1 = self._group.plan()
            # print "============ Waiting while RVIZ displays plan1..."
            self._group.go(wait=True)
            rospy.sleep(2)

            self._group.clear_pose_targets()
            current_joint_vals = self._group.get_current_joint_values()
            current_joint_vals[0] -= .5
            rospy.logwarn(current_joint_vals)
            self._group.set_joint_value_target(current_joint_vals)
            self._group.plan()
            self._group.go(wait=True)

            self._group.clear_pose_targets()
            current_joint_vals = self._group.get_current_joint_values()
            current_joint_vals[0] += .5
            rospy.logwarn(current_joint_vals)
            self._group.set_joint_value_target(current_joint_vals)
            self._group.plan()
            self._group.go(wait=True)

            self._group.clear_pose_targets()

            # current_joint_vals = self._group.get_current_joint_values()
            current_joint_vals = self._warehouse_name_get_srv("dummy_state", self._robot_name).state

            # current_joint_vals[0] += .5
            rospy.logwarn("===================")

            rospy.logwarn(current_joint_vals.joint_state.position)
            self._group.set_joint_value_target(list(current_joint_vals.joint_state.position))
            self._group.plan()
            self._group.go(wait=True)

            rospy.logerr("all work and no fun has made UR5 a dull robot")


if __name__ == "__main__":
    rospy.init_node("ur5_demo_node", anonymous=True)
    commander = Ur5Commander()
    commander.get_info()
    while not rospy.is_shutdown():
        rospy.spin()
    moveit_commander.roscpp_shutdown()


