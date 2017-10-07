#!/usr/bin/env python
import rospy
import unittest
import itertools
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

PKG = "navigator_gazebo"


class TestSimIntegration(unittest.TestCase):

    def __init__(self, *args):
        super(TestSimIntegration, self).__init__(*args)
        rospy.sleep(1)
        # subscribe to odometry topics
        self.odom_pos_msg = []
        self.odom_ori_msg = []
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.absodom_pos_msg = []
        self.absodom_ori_msg = []
        rospy.Subscriber("/absodom", Odometry, self.absodom_cb)
        # subscribe to camera info topics
        self.cam_info_msg = []
        self.cam_image_msg = []
        topics_cam_info = [
            "/camera/front/right/camera_info",
            "/camera/front/left/camera_info",
            "/camera/down/camera_info",
            "/camera/starboard/camera_info"]
        for topic in topics_cam_info:
            rospy.Subscriber(topic, CameraInfo, self.cam_info_cb)
        # subscribe to camera image topics
        self.image_msg = []
        topics_image = [
            "/camera/front/right/image_color",
            "/camera/front/left/image_color",
            "/camera/down/image_color",
            "/camera/starboard/image_color"]
        for topic in topics_image:
            rospy.Subscriber(topic, Image, self.cam_image_cb)
        # subscribe to pointcloud topic
        self.pc_info_msg = []
        self.pc_msg = []
        rospy.Subscriber("/velodyne_points", PointCloud2, self.points_cb)

    def odom_cb(self, msg):
        self.odom_pos_msg.append(msg.pose.pose.position.x)
        self.odom_pos_msg.append(msg.pose.pose.position.y)
        self.odom_pos_msg.append(msg.pose.pose.position.z)
        self.odom_ori_msg.append(msg.pose.pose.orientation.x)
        self.odom_ori_msg.append(msg.pose.pose.orientation.y)
        self.odom_ori_msg.append(msg.pose.pose.orientation.z)
        self.odom_ori_msg.append(msg.pose.pose.orientation.w)

    def test_odom(self):
        timeout = rospy.Time.now() + rospy.Duration(1)
        while ((self.odom_pos_msg == False or self.odom_ori_msg == False)
               and rospy.Time.now() < timeout):
            rospy.Sleep(0.01)
        self.assertTrue(self.odom_pos_msg and self.odom_ori_msg)
        initial_pos = [-1.2319, 0.0, 0.0]
        initial_ori = [0.0, 0.0, 0.0, 1.0]
        self.verify_pos_ori(
            self.odom_pos_msg,
            initial_pos,
            self.odom_ori_msg,
            initial_ori,
            "/odom")

    def absodom_cb(self, msg):
        self.absodom_pos_msg.append(msg.pose.pose.position.x)
        self.absodom_pos_msg.append(msg.pose.pose.position.y)
        self.absodom_pos_msg.append(msg.pose.pose.position.z)
        self.absodom_ori_msg.append(msg.pose.pose.orientation.x)
        self.absodom_ori_msg.append(msg.pose.pose.orientation.y)
        self.absodom_ori_msg.append(msg.pose.pose.orientation.z)
        self.absodom_ori_msg.append(msg.pose.pose.orientation.w)

    def test_absodom(self):
        timeout = rospy.Time.now() + rospy.Duration(1)
        while ((self.absodom_pos_msg == False or self.absodom_ori_msg ==
                False) and rospy.Time.now() < timeout):
            rospy.Sleep(0.01)
        self.assertTrue(self.absodom_pos_msg and self.absodom_ori_msg)
        initial_pos = [743789.637462, -5503821.62581, 3125622.25266]
        initial_ori = [0.0, 0.0, 0.0, 1.0]
        self.verify_pos_ori(
            self.absodom_pos_msg,
            initial_pos,
            self.absodom_ori_msg,
            initial_ori,
            "/absodom")

    def verify_pos_ori(self, pos, initial_pos, ori, initial_ori, topic):
        # make assertions
        for actual, initial in itertools.izip(pos, initial_pos):
            self.assertAlmostEqual(
                actual, initial, places=4, msg=(
                    "Error: {} position is: {} should be {}".format(
                        topic, actual, initial)))
        for actual, initial in itertools.izip(ori, initial_ori):
            self.assertEqual(
                actual, initial, msg=(
                    "Error: {} orientation is: {} should be {}".format(
                        topic, actual, initial)))

    def cam_info_cb(self, msg):
        self.cam_info_msg.append([msg.width, msg.height])

    def cam_image_cb(self, msg):
        self.cam_image_msg.append(msg.data)

    def test_image(self):
        timeout = rospy.Time.now() + rospy.Duration(1)
        while ((self.cam_info_msg == False or self.cam_image_msg ==
                False) and rospy.Time.now() < timeout):
            rospy.Sleep(0.01)
        self.assertTrue(self.cam_info_msg and self.cam_image_msg)
        initial_res = [960, 600]
        self.verify_info(self.cam_info_msg, initial_res, "/camera")
        self.verify_not_empty(self.cam_image_msg)

    def points_cb(self, msg):
        self.pc_info_msg.append([msg.width, msg.height])
        self.pc_msg.append(msg.data)
        self.pc_msg.append(msg.data)

    def test_pointcloud(self):
        timeout = rospy.Time.now() + rospy.Duration(1)
        while((self.pc_info_msg == False or self.pc_msg == False) and rospy.Time.now() < timeout):
            rospy.Sleep(0.01)
        self.assertTrue(self.pc_info_msg and self.pc_msg)
        initial_res = [209, 1]
        self.verify_info(self.pc_info_msg, initial_res, "/velodyne_points")
        self.verify_not_empty(self.pc_msg)

    def verify_not_empty(self, data_lists):
        self.assertTrue(self.is_not_empty(data_lists))

    def is_not_empty(self, data_lists):
        for data_list in data_lists:
            if len(data_list) == 0:
                return False
        return True

    def verify_info(self, res_info, initial_info, topic):
        for msg in res_info:
            for actual_dim, initial_dim in itertools.izip(msg, initial_info):
                self.assertEqual(
                    actual_dim, initial_dim, msg=(
                        "Error: {} is: {} should be {}".format(
                            topic, actual_dim, initial_dim)))


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_sim_integration', anonymous=True)
    print "name", rospy.get_name()
    print "namespace", rospy.get_namespace()
    rostest.rosrun(PKG, 'sim_integration_test', TestSimIntegration)
