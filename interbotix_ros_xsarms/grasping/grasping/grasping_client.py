import rclpy
from threading import Thread
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
from interbotix_xs_msgs.srv import MotorGains
from geometry_msgs.msg import TransformStamped
from raya_grasp_msgs.srv import GetGripperPoses
from raya_grasp_msgs.srv import GetGripperPosesPlace
from raya_cv_msgs.msg import DetectionsAT
from raya_arms_msgs.action import ArmJointPlanner, ArmPosePlanner, ArmNamePosPlanner


import time

import tf_transformations
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

GET_TRANSFORM_POSITION = [
    7 * np.pi / 180,
    19 * np.pi / 180,
    33* np.pi / 180,
    0 * np.pi / 180,
    35 * np.pi / 180,
    0 * np.pi / 180,
]

SLEEP_POSITION = [
    0 * np.pi / 180,
    -107 * np.pi / 180,
    90* np.pi / 180,
    0 * np.pi / 180,
    20 * np.pi / 180,
    0 * np.pi / 180,
]

HOME_POSITION = [
    0.0,0.0,0.0,0.0,0.0,0.0
]

PID_GAINS_ARMS={
"cmd_type":'group',         
"name":'arm',          
"kp_pos":800,           
"ki_pos":50,           
"kd_pos":0,           
"k1":0,           
"k2":0,           
"kp_vel":100,           
"ki_vel":1920,           }

class GraspingClient(Node):
    def __init__(self):
        super().__init__("grasping_client")
        self.cli = self.create_client(GetGripperPoses, "raya/grasp/get_gripper_poses")
        self.cli_pids = self.create_client(MotorGains, "set_motor_pid_gains")
        self.cli_place = self.create_client(GetGripperPosesPlace, "raya/grasp/get_gripper_poses_place")
        self._pose_client = ActionClient(self, ArmPosePlanner, "locobot_pose_action")
        self._name_client = ActionClient(self, ArmNamePosPlanner, "/locobot_name_pos_action")
        self._joint_client = ActionClient(self, ArmJointPlanner, "locobot_joint_action")
        self.tf_br = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.apriltag_tf = None
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subs_april_tag = self.create_subscription(
            DetectionsAT,
            "/raya/cv/detectors/tag/detections2D",
            self.callback_apriltag,
            qos_policy,
        )
        self.read_april_tags = False

    def create_subs_april_tag(self):
        self.read_april_tags = True

    def callback_apriltag(self, msg):
        for tag in msg.detections:
            if tag.tag_id == 42 and self.read_april_tags:
                T = tag.pose_t
                R = [
                    tag.pose_r_a,
                    tag.pose_r_b,
                    tag.pose_r_c,
                ]
                A = np.eye(4)
                A[:3, :3] = R
                A[:-1, -1] = T
                q = tf_transformations.quaternion_from_matrix(A)
                transform = tf_transformations.concatenate_matrices(
                    tf_transformations.translation_matrix(T),
                    tf_transformations.quaternion_matrix(q),
                )
                inversed_transform = tf_transformations.inverse_matrix(transform)
                T = tf_transformations.translation_from_matrix(inversed_transform)
                q = tf_transformations.quaternion_from_matrix(inversed_transform)
                self.apriltag_tf = TransformStamped()
                self.apriltag_tf.header.stamp = self.get_clock().now().to_msg()
                self.apriltag_tf.header.frame_id = "april_tag"
                self.apriltag_tf.child_frame_id = "rs_d435_color_optical_frame"
                self.apriltag_tf.transform.rotation.x = q[0]
                self.apriltag_tf.transform.rotation.y = q[1]
                self.apriltag_tf.transform.rotation.z = q[2]
                self.apriltag_tf.transform.rotation.w = q[3]
                self.apriltag_tf.transform.translation.x = T[0]
                self.apriltag_tf.transform.translation.y = T[1]
                self.apriltag_tf.transform.translation.z = T[2]
                self.destroy_subscription(self.subs_april_tag)

    def create_tf(self):
        now = rclpy.time.Time()
        trans = self.tf_buffer.lookup_transform(
             "rs_d435_color_optical_frame","rs_d435_link", now
        )
        T1=[self.apriltag_tf.transform.translation.x,self.apriltag_tf.transform.translation.y,self.apriltag_tf.transform.translation.z]
        T2=[trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z]
        q1=[self.apriltag_tf.transform.rotation.x,self.apriltag_tf.transform.rotation.y,self.apriltag_tf.transform.rotation.z,self.apriltag_tf.transform.rotation.w]
        q2=[trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w]

        A = np.eye(4)
        A[:3, :3] = tf_transformations.quaternion_matrix(q1)[:3, :3]
        A[:-1, -1] = T1
        B = np.eye(4)
        B[:3, :3] = tf_transformations.quaternion_matrix(q2)[:3, :3]
        B[:-1, -1] = T2
        transform = np.matmul(A,B)
        
        T = tf_transformations.translation_from_matrix(transform)
        q = tf_transformations.quaternion_from_matrix(transform)
        trans.header.stamp = self.get_clock().now().to_msg()
        trans.header.frame_id = "april_tag"
        trans.child_frame_id = "rs_d435_link"
        trans.transform.rotation.x = q[0]
        trans.transform.rotation.y = q[1]
        trans.transform.rotation.z = q[2]
        trans.transform.rotation.w = q[3]
        trans.transform.translation.x = T[0]
        trans.transform.translation.y = T[1]
        trans.transform.translation.z = T[2]
        self.tf_buffer.set_transform_static(trans, "try")
        trans = self.tf_buffer.lookup_transform(
             "wx250s/base_arm_link","rs_d435_link", now
        )
        self.tf_br.sendTransform(trans)
        return True

    def call_service(self):
        while not self.cli.wait_for_service(timeout_sec=0.1):
            self.get_logger().info("service not available, waiting again...")
        self.req = GetGripperPoses.Request()
        self.req.object_name = "bottle"
        self.req.tries = 50
        try:
            self.future = self.cli.call(self.req)
        except:
            print("service fail")

    def call_service_place(self, height):
        while not self.cli_place.wait_for_service(timeout_sec=0.1):
            self.get_logger().info("service not available, waiting again...")
        self.req = GetGripperPosesPlace.Request()
        self.req.family = "tag36h11"
        self.req.id = 42
        self.req.height=height
        self.req.tries=50
        try:
            self.future = self.cli_place.call(self.req)
        except:
            print("service fail")
    
    def call_service_PIDs(self):
        while not self.cli_pids.wait_for_service(timeout_sec=0.1):
            self.get_logger().info("service pids not available, waiting again...")
        self.req = MotorGains.Request()
        self.req.cmd_type=PID_GAINS_ARMS["cmd_type"]
        self.req.name=PID_GAINS_ARMS["name"]
        self.req.kp_pos=PID_GAINS_ARMS["kp_pos"]
        self.req.ki_pos=PID_GAINS_ARMS["ki_pos"]
        self.req.kd_pos=PID_GAINS_ARMS["kd_pos"]
        self.req.k1=PID_GAINS_ARMS["k1"]
        self.req.k2=PID_GAINS_ARMS["k2"]
        self.req.kp_vel=PID_GAINS_ARMS["kp_vel"]
        self.req.ki_vel=PID_GAINS_ARMS["ki_vel"]
        try:
            self.future_pid = self.cli_pids.call(self.req)
        except:
            print("service fail")

    def move_arm(self, pose, cartesian_path=True):
        goal_msg = ArmPosePlanner.Goal()
        goal_msg.goal_pose = pose
        goal_msg.arm = "interbotix_arm"
        goal_msg.cartesian_path = cartesian_path

        self._pose_client.wait_for_server()

        return self._pose_client.send_goal(goal_msg)

    def move_arm_joints(self, joints):
        goal_msg = ArmJointPlanner.Goal()
        goal_msg.position = joints
        goal_msg.arm = "interbotix_arm"

        self._joint_client.wait_for_server()

        return self._joint_client.send_goal(goal_msg)
    
    def move_arm_name(self, name):
        goal_msg = ArmNamePosPlanner.Goal()
        goal_msg.name = name
        goal_msg.arm = "interbotix_arm"

        self._name_client.wait_for_server()

        return self._name_client.send_goal(goal_msg)

    def move_gripper(self, width_object):
        goal_msg = ArmJointPlanner.Goal()
        distance = (width_object / 2) - 0.0125
        if distance < 0.011:
            distance = 0.011
        print(f"distancia: {distance}")
        goal_msg.position = [distance, -distance]
        goal_msg.arm = "interbotix_gripper"

        self._joint_client.wait_for_server()

        return self._joint_client.send_goal(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    nxt = True
    node = GraspingClient()
    thread = Thread(target=rclpy.spin, args=(node,))
    thread.start()

    # if nxt:
    #     count=0
    #     while count<2:
    #         node.call_service_PIDs()
    #         response = node.future_pid
    #         count=count+1
    #         time.sleep(0.1)

    # if nxt:
    #     print("going tf position")
    #     get_transform = node.move_arm_joints(GET_TRANSFORM_POSITION)
    #     if get_transform.result.error == 0:
    #         nxt = True
    #     else:
    #         print("fail transformation position")
    #         nxt = False 
    # if nxt:
    #     node.create_subs_april_tag()
    #     while node.apriltag_tf == None:
    #         time.sleep(0.1)
    #         print("waiting subscription")
    #     node.create_tf()
    
    # if nxt:
    #     time.sleep(1)
    #     print("going home position")
    #     home = node.move_arm_name("Sleep")
    #     if home.result.error == 0:
    #         nxt = True
    #     else:
    #         print("fail home position")
    #         nxt = False 

    if nxt:
        print("get position")
        time.sleep(0.4)
        node.call_service()
        response = node.future
        count=0
        nxt=False
        while count<5 and nxt ==False:
            if (
                response.pre_grasp_pose.position.x != 0
                and response.pre_grasp_pose.position.y != 0
                and response.pre_grasp_pose.position.z != 0
            ):
                nxt = True
            else:
                count =count+1
                print("fail get bottle pose")
                nxt = False

    if nxt:
        print("pre grasp position")
        time.sleep(0.4)
        pre_grasp = node.move_arm(response.pre_grasp_pose,False)
        if pre_grasp.result.error == 0:
            nxt = True
        else:
            print("fail pre grasp")
            nxt = False
    
    
    if nxt:
        print("open grip")
        time.sleep(0.4)
        open_grip = node.move_gripper(0.101)
        if open_grip.result.error == 0:
            nxt = True
        else:
            print("fail open")
            nxt = False
    if nxt:
        print("grasp")
        time.sleep(0.4)
        grasp = node.move_arm(response.grasp_pose,True)
        if grasp.result.error == 0:
            nxt = True
        else:
            print("fail grasp")
            nxt = False
    if nxt:
        time.sleep(0.4)
        print("close_grip")
        close_grip = node.move_gripper(response.width_object)
        if close_grip.result.error == 0:
            nxt = True
        else:
            print("fail close")
            nxt = False
    if nxt:
        time.sleep(0.4)
        post_grasp = node.move_arm(response.post_grasp_pose)
        if post_grasp.result.error == 0:
            nxt = True
        else:
            print("fail post grasp")
            nxt = False
    if nxt:
        time.sleep(2)
        print("going sleep position")
        sleep = node.move_arm_name("Sleep")
        if sleep.result.error == 0:
            nxt = True
        else:
            print("fail sleep position")
            nxt = False 
    if nxt:
        print("get position")
        time.sleep(1)
        node.call_service_place(response.height_object)
        response = node.future
        count=0
        nxt=False
        while count<5 and nxt ==False:
            if (
                response.pre_place_pose.position.x != 0
                and response.pre_place_pose.position.y != 0
                and response.pre_place_pose.position.z != 0
            ):
                nxt = True
            else:
                count =count+1
                print("fail apriltag pose")
                nxt = False

    if nxt:
        print("pre place position")
        time.sleep(1)
        pre_place = node.move_arm(response.pre_place_pose, False)
        if pre_place.result.error == 0:
            nxt = True
        else:
            print("fail pre place")
            nxt = False
    
    if nxt:
        print("place")
        time.sleep(2)
        place = node.move_arm(response.place_pose,True)
        if place.result.error == 0:
            nxt = True
        else:
            print("fail place")
            nxt = False

    if nxt:
        print("open grip")
        time.sleep(1)
        open_grip = node.move_gripper(0.101)
        if open_grip.result.error == 0:
            nxt = True
        else:
            print("fail open")
            nxt = False
            
    if nxt:
        time.sleep(0.4)
        post_place = node.move_arm(response.post_place_pose)
        if post_place.result.error == 0:
            nxt = True
        else:
            print("fail post place")
            nxt = False
    if nxt:
        time.sleep(2)
        print("going sleep position")
        sleep = node.move_arm_name("Sleep1")
        if sleep.result.error == 0:
            nxt = True
        else:
            print("fail sleep position")
            nxt = False 
    # if nxt:
    #     time.sleep(0.4)
    #     open_grip = node.move_gripper(0.101)
    #     print("melooo!!")

  

if __name__ == "__main__":
    main()
