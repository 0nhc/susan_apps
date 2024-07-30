#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import actionlib
import moveit_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import control_msgs.msg
import trajectory_msgs.msg
import math
import sys
import time
import pickle

NUM_JOINTS = 7

class TrajPoint:
    def __init__(self, px=0.0, py=0.0, pz=0.0, ox=0.0, oy=0.0, oz=0.0, ow=1.0) -> None:
        self.px = px
        self.py = py
        self.pz = pz
        self.ox = ox
        self.oy = oy
        self.oz = oz
        self.ow = ow

class Traj:
    def __init__(self, num_points=1) -> None:
        self.data = []
        self.num_points = num_points
        for i in range(self.num_points):
            self.data.append(TrajPoint())

def dist(p, q):
    return math.sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = math.fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= math.cos(tolerance / 2.0)

    return True

class SusanMoveit(object):
    def __init__(self) -> None:
        super(SusanMoveit, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("susan_moveit", anonymous=True)
        self.init_susan_variables()

        # Action Client for Moving Joints
        # self.joint_command_publisher = rospy.Publisher('/susan_arm_group_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
        self.ros_control_action_client = actionlib.SimpleActionClient('/susan_arm_group_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
        self.ros_control_action_client.wait_for_server()
        
        self.ros_control_action_goal = control_msgs.msg.FollowJointTrajectoryActionGoal()
        self.ros_control_action_goal = control_msgs.msg.FollowJointTrajectoryGoal()
        # self.ros_control_action_goal.trajectory.header.stamp = rospy.Time.now()
        self.ros_control_action_goal.trajectory.header.frame_id = "susan_arm_group_action_goal.frame_id"
        self.ros_control_action_goal.goal_time_tolerance = rospy.rostime.Duration(1)

        # Init Move Group
        self.group_name = "susan_arm_group"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # Initialize the timer
        self.timer_freq = 100  # Hz
        self.timer = rospy.Timer(rospy.Duration(1.0/self.timer_freq), self.main_loop)

        # Initialize the IK service client
        self.ik_service = rospy.ServiceProxy('/compute_ik', moveit_msgs.srv.GetPositionIK)

        # Subscriber for current joint state
        self.joint_state_sub = rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.joint_state_callback)




        

    def joint_state_callback(self, msg):
        self.joint_state = msg
        if(self.init == False):
           self.init = True

    def pose_command_callback(self, msg):
        self.pose_command.position.x = msg.position.x
        self.pose_command.position.y = msg.position.y
        self.pose_command.position.z = msg.position.z

    def init_susan_variables(self):
        self.pose_state = geometry_msgs.msg.Pose()
        # Goal State: Ready
        self.pose_command = geometry_msgs.msg.Pose()
        self.joint_state = sensor_msgs.msg.JointState()
        self.base_link_name = "link1"
        self.end_effector_name = "hand"
        self.init = False

    def calculate_ik(self, target_pose):
        # Solve Inverse Kinematics
        service_request = moveit_msgs.msg.PositionIKRequest()
        service_request.group_name = self.group_name
        service_request.robot_state = moveit_msgs.msg.RobotState()
        service_request.robot_state.joint_state = self.joint_state
        # service_request.ik_link_name = self.end_effector_name
        service_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        service_request.pose_stamped.header.frame_id = self.base_link_name
        service_request.pose_stamped.header.stamp = rospy.Time.now()
        service_request.pose_stamped.pose = target_pose
        # service_request.constraints.joint_constraints.append(moveit_msgs.msg.JointConstraint)
        # service_request.constraints.joint_constraints[0].joint_name = "right_arm_joint4"
        # service_request.constraints.joint_constraints[0].position = 0.0 # the bound to be achieved is [position - tolerance_below, position + tolerance_above]
        # service_request.constraints.joint_constraints[0].tolerance_above = 2.50
        # service_request.constraints.joint_constraints[0].tolerance_below = 0.01
        # service_request.constraints.joint_constraints[0].weight = 0.5
        # service_request.constraints.joint_constraints.append(moveit_msgs.msg.JointConstraint)
        # service_request.constraints.joint_constraints[1].joint_name = "right_arm_joint3"
        # service_request.constraints.joint_constraints[1].position = 0.0 # the bound to be achieved is [position - tolerance_below, position + tolerance_above]
        # service_request.constraints.joint_constraints[1].tolerance_above = 1.00
        # service_request.constraints.joint_constraints[1].tolerance_below = 0.05
        # service_request.constraints.joint_constraints[1].weight = 0.3
        service_request.timeout = rospy.Duration(3.0)
        service_request.avoid_collisions = True

        try:
            resp = self.ik_service(ik_request = service_request)
            if resp.error_code.val == resp.error_code.SUCCESS:
                rospy.loginfo("IK service succeeded.")
                return resp
            else:
                rospy.logwarn("IK service failed with error code: "+str(resp.error_code))
                return None
        except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)
            return None
        
    def print_pose_state(self):
        self.pose_state = self.move_group.get_current_pose(self.end_effector_name).pose
        rospy.loginfo(str(self.pose_state))

    def go_to_joint_state(self, joint_names,  joint_goals):
        if type(joint_goals) is list:
            move_group = self.move_group
            joint_goal = move_group.get_current_joint_values()
            for i in range(NUM_JOINTS):
                joint_goal[i] = joint_goals[i]
            move_group.go(joint_goal, wait=True)
            move_group.stop()
            current_joints = move_group.get_current_joint_values()
            return all_close(joint_goal, current_joints, 0.01)
        else:
            rospy.logwarn("Invalid Input for Function go_To_joint_state. Type must be List.")

    def go_to_joint_state_ros_control(self, joint_names,  joint_goals):
        if type(joint_goals) is list:
            self.ros_control_action_goal.trajectory.header.stamp = rospy.Time.now()
            self.ros_control_action_goal.trajectory.joint_names = joint_names
            self.ros_control_action_goal.trajectory.points[0].positions = joint_goals
            self.ros_control_action_client.send_goal(self.ros_control_action_goal)
            self.ros_control_action_client.wait_for_result()
            rospy.loginfo(str(self.ros_control_action_client.get_result()))
            return self.ros_control_action_client.get_result()

    def main_loop(self, event):
        if(self.init == True):
            pass

    def follow_goal_trajectory(self, path):
        while(self.init == False):
            pass
        with open(path, 'rb') as f:
            self.goal_trajectory = pickle.load(f)
        for idx in range(self.goal_trajectory["num_points"]):
            pose_point = self.goal_trajectory["data"][idx]
            rospy.loginfo("px: "+str(pose_point.px)+", py: "+str(pose_point.py)+", pz: "+str(pose_point.pz)+
                        ", ox: "+str(pose_point.ox)+", oy: "+str(pose_point.oy)+", oz: "+str(pose_point.oz)+", ow: "+str(pose_point.ow))
            self.pose_command.position.x = pose_point.px
            self.pose_command.position.y = pose_point.py
            self.pose_command.position.z = pose_point.pz
            self.pose_command.orientation.x = pose_point.ox
            self.pose_command.orientation.y = pose_point.oy
            self.pose_command.orientation.z = pose_point.oz
            self.pose_command.orientation.w = pose_point.ow
            ik_result = self.calculate_ik(self.pose_command)
            joint_names = ik_result.solution.joint_state.name
            joint_commands = list(ik_result.solution.joint_state.position)

            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.time_from_start = rospy.rostime.Duration(idx*0.3)
            point.positions = joint_commands
            self.ros_control_action_goal.trajectory.points.append(point)

        self.go_to_joint_state_ros_control(joint_names, joint_commands)


    def run(self):
        rospy.spin()
    
def main():
    try:
        susan = SusanMoveit()
        # susan.print_pose_state()
        susan.follow_goal_trajectory("/home/hzx/trajectory.pickle")

    except rospy.ROSInterruptException:
        return
    
    except KeyboardInterrupt:
        return
    
if __name__ == "__main__":
    main()