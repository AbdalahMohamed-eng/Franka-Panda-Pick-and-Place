#!/usr/bin/env python3
import sys
import math
import rospy
import moveit_commander
import geometry_msgs.msg
import tf.transformations
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

class SmartPickAndPlace:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("stone_picker", anonymous=False)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("panda_arm")
        self.hand_group = moveit_commander.MoveGroupCommander("panda_hand")

        self.arm_group.set_planning_time(15.0)
        self.arm_group.set_max_velocity_scaling_factor(0.3)
        self.arm_group.set_max_acceleration_scaling_factor(0.3)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.scan_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
        self.home_joints = [0.0, -math.pi/4, 0.0, -3*math.pi/4, 0.0, math.pi/2, math.pi/4]
        self.drop_pos = geometry_msgs.msg.Point(0.4, 0.4, 0.5) 
        self.quat_down = tf.transformations.quaternion_from_euler(math.pi, 0, 0)

        self.target_pose = None
        self.target_acquired = False

        rospy.Subscriber("/stone_position", PointStamped, self.stone_callback)
        self.add_table_collision()
        rospy.sleep(1.0)
        self.execute_mission()

    def add_table_collision(self):
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "panda_link0"
        p.pose.position.x = 0.5
        p.pose.position.y = 0.0
        p.pose.position.z = 0.10  
        p.pose.orientation.w = 1.0
        self.scene.add_box("table", p, (0.5, 0.7, 0.20))
        rospy.sleep(1.0)

    def stone_callback(self, msg):
        if self.target_acquired:
            return
        try:
            transform = self.tf_buffer.lookup_transform("panda_link0", 
                                                        msg.header.frame_id, 
                                                        rospy.Time(0), 
                                                        rospy.Duration(1.0))
            p_transformed = tf2_geometry_msgs.do_transform_point(msg, transform)
            self.target_pose = p_transformed.point
            self.target_acquired = True
            rospy.loginfo(f"Target Found: x={self.target_pose.x:.3f}, y={self.target_pose.y:.3f}")
        except Exception as e:
            rospy.logwarn(f"TF Error: {e}")

    def go_joints(self, joints):
        self.arm_group.set_joint_value_target(joints)
        self.arm_group.go(wait=True)

    def open_gripper(self):
        self.hand_group.set_joint_value_target([0.04, 0.04])
        self.hand_group.go(wait=True)

    def close_gripper(self):
        rospy.loginfo("Closing Gripper...")
        self.hand_group.set_joint_value_target([0.0, 0.0])
        self.hand_group.go(wait=True)
        rospy.sleep(2.0) # Wait for magnet to lock

    def create_pose(self, x, y, z):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = self.quat_down[0]
        pose.orientation.y = self.quat_down[1]
        pose.orientation.z = self.quat_down[2]
        pose.orientation.w = self.quat_down[3]
        return pose

    def go_straight(self, pose):
        waypoints = [pose]
        (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, True)
        if fraction < 0.5:
            return False
        self.arm_group.execute(plan, wait=True)
        return True

    def execute_mission(self):
        rospy.loginfo("=== STARTING MISSION ===")
        self.go_joints(self.scan_joints)
        self.open_gripper()
        
        rospy.loginfo("Waiting for vision...")
        while not self.target_acquired:
            rospy.sleep(0.5)

        # 1. HOVER
        hover_pose = self.create_pose(self.target_pose.x, self.target_pose.y, 0.50)
        self.arm_group.set_pose_target(hover_pose)
        self.arm_group.go(wait=True)

        # 2. GRASP (Z=0.422 is the proven safe depth)
        grasp_z = 0.422 
        grasp_pose = self.create_pose(self.target_pose.x, self.target_pose.y, grasp_z)
        
        rospy.loginfo(f"Descending to Z={grasp_z:.3f}...")
        self.arm_group.set_max_velocity_scaling_factor(0.1)
        
        if not self.go_straight(grasp_pose):
            self.arm_group.set_pose_target(grasp_pose)
            self.arm_group.go(wait=True)

        # 3. PICK
        self.close_gripper()

        # 4. LIFT
        rospy.loginfo("Lifting...")
        self.arm_group.set_max_velocity_scaling_factor(0.3)
        self.go_straight(hover_pose)

        # 5. DROP
        self.go_joints(self.scan_joints) 
        drop_pose_msg = self.create_pose(self.drop_pos.x, self.drop_pos.y, self.drop_pos.z)
        self.arm_group.set_pose_target(drop_pose_msg)
        self.arm_group.go(wait=True)
        self.open_gripper()
        self.go_joints(self.home_joints)
        rospy.loginfo("Done.")

if __name__ == "__main__":
    try:
        SmartPickAndPlace()
    except rospy.ROSInterruptException:
        pass