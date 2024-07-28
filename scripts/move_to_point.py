#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import math
import sys
import time

NUM_JOINTS = 7

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

        self.init_susan_variables()

    def joint_state_callback(self, msg):
        self.joint_state = msg
        if(self.init == False):
           self.init = True

    def init_susan_variables(self):
        self.pose_state = geometry_msgs.msg.Pose()
        self.pose_command = geometry_msgs.msg.Pose()
        self.joint_state = sensor_msgs.msg.JointState()
        self.base_link_name = "link1"
        self.end_effector_name = "hand"
        self.init = False

    def calculate_ik(self, target_pose):
        # # Create an IK request message
        # ik_request = moveit_msgs.srv.GetPositionIKRequest()
        # ik_request.ik_request.group_name = self.group_name
        # ik_request.ik_request.pose_stamped.header.frame_id = self.base_link_name
        # ik_request.ik_request.pose_stamped.pose = target_pose

        # # Call the IK service
        # try:
        #     ik_response = self.ik_service(ik_request)
        #     if ik_response.error_code.val == ik_response.error_code.SUCCESS:
        #         rospy.logwarn(f"IK service succeeded with results: {ik_response.solution.joint_state.position}")
        #         return ik_response.solution.joint_state.position
        #     else:
        #         rospy.logwarn(f"IK service failed with error code: {ik_response.error_code}")
        #         return None
        # except rospy.ServiceException as e:
        #     rospy.logerr("Service call failed: %s", e)
        #     return None

        service_request = moveit_msgs.msg.PositionIKRequest()
        service_request.group_name = self.group_name
        service_request.robot_state = moveit_msgs.msg.RobotState()
        service_request.robot_state.joint_state = self.joint_state
        # service_request.ik_link_name = self.end_effector_name
        service_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        service_request.pose_stamped.header.frame_id = self.base_link_name
        service_request.pose_stamped.header.stamp = rospy.Time.now()
        service_request.pose_stamped.pose = target_pose
        service_request.constraints.joint_constraints.append(moveit_msgs.msg.JointConstraint)
        service_request.constraints.joint_constraints[0].joint_name = "right_arm_joint3"
        # the bound to be achieved is [position - tolerance_below, position + tolerance_above]
        service_request.constraints.joint_constraints[0].position = 0.0
        service_request.constraints.joint_constraints[0].tolerance_above = 0.001
        service_request.constraints.joint_constraints[0].tolerance_below = 0.001
        service_request.constraints.joint_constraints[0].weight = 0.2
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

    def go_to_joint_state(self, goals):
        if type(goals) is list:
            move_group = self.move_group
            joint_goal = move_group.get_current_joint_values()
            for i in range(NUM_JOINTS):
                joint_goal[i] = goals[i]
            move_group.go(joint_goal, wait=True)
            move_group.stop()
            current_joints = move_group.get_current_joint_values()
            return all_close(joint_goal, current_joints, 0.01)
        else:
            rospy.logwarn("Invalid Input for Function go_To_joint_state. Type must be List.")

    def main_loop(self, event):
        if(self.init == True):
            self.print_pose_state()
            pose = geometry_msgs.msg.Pose()
            pose.position.x = 0.23020871736041854
            pose.position.y = -0.09804201997659427
            pose.position.z = -0.2747095401812456
            pose.orientation.x = 0.7070666262443575
            pose.orientation.y = -0.7071468204090466
            pose.orientation.z = -0.00024981334063552845
            pose.orientation.w = 0.00031309757866667385
            ik_result = self.calculate_ik(pose)
            joint_names = ik_result.solution.joint_state.name
            joint_commands = list(ik_result.solution.joint_state.position)
            rospy.loginfo("IK names: "+str(joint_names))
            rospy.loginfo("IK results: "+str(joint_commands))
            self.go_to_joint_state(joint_commands)

    def run(self):
        rospy.spin()
    
def main():
    try:
        susan = SusanMoveit()
        susan.run()

    except rospy.ROSInterruptException:
        return
    
    except KeyboardInterrupt:
        return
    
if __name__ == "__main__":
    main()