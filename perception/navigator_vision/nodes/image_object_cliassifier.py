#!/usr/bin/env python
import numpy as np
import rospy
import image_geometry
import mil_tools
from sensor_msgs.msg import CameraInfo, Image
import tf
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryResponse
from mil_msgs.msg import PerceptionObject


class ImageGetter(object):
    def __init__(self, topic_name):
        self.sub = mil_tools.Image_Subscriber(topic_name, self.get_image)

        print ('getting topic', topic_name)
        self.frame = None
        self.done = False

        self.camera_info = self.sub.wait_for_camera_info()
        self.img_geom = image_geometry.PinholeCameraModel()
        self.img_geom.fromCameraInfo(self.camera_info)

    def get_image(self, msg):
        self.frame = msg
        self.done = True

class ObjectImageClassifier(object):
    def __init__(self):
        self.camera_topics = ["/camera/front/left/image_rect_color", "/camera/seecam/image_rect_color"]
        self.camera_tfs = ["/front_left_cam", "/seecam"]

        self.cameras = [ImageGetter(p) for p in self.camera_topics]

        while None in [p.img_geom for p in self.cameras] and not rospy.is_shutdown():
            print (self.cameras)
            print ("Waiting frames...")
            rospy.sleep(1)

        rospy.wait_for_service('/database/requests')
        self.DBQuery_proxy = rospy.ServiceProxy('/database/requests', ObjectDBQuery)

        while not rospy.is_shutdown():
            try:
                [tf_listener.waitForTransform("enu", t, rospy.Time(), rospy.Duration(5)) for t in self.camera_tfs]
                # get a list for all points and all labels for each camera 
                info = [self.get_label_and_points(c.frame) for c in self.cameras]

                # get pose for all the cameras
                enu_cam_tf = [tf_listener.lookupTransform("enu", self.camera_tfs[i], self.cameras[i].sub.last_image_time) for i in range(len(self.cameras))]

                cam_p, cam_q = zip(*enu_cam_tf)
                cam_p = [np.array(p) for p in cam_p]
                cam_r = [tf.transformations.quaternion_matrix(q)[:3, :3] for q in cam_q]
                all_objects = None
                try:
                    all_objects = self.DBQuery_proxy(name="all", cmd="").objects
                    # print("called")
                    # print(all_objects)
                except rospy.ServiceException, e:
                    print ("Couldn't call database")
                    rospy.sleep(5)
                    continue
                if all_objects is None:
                    print ("Didn't find any objects")
                    rospy.sleep(5)
                    continue
                for i in range(len(self.cameras)):
                    points, labels = info[i]
                    for j in range(len(points)):
                        ray = self.get_ray_in_enu(points[j], self.cameras[i].img_geom, cam_r[i])
                        if ray is None:
                            continue
                        # print("hey")
                        o = self.find_closest_object_given_ray(all_objects, ray, cam_p[i])
                        if o is not None:
                            self.set_label_for_object(labels[j], o)
                    rospy.sleep(2)
                        # find intersection in database and pray it works
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, TypeError) as e:
                print e
                rospy.logwarn("TF link not found.")
                rospy.sleep(.5)
                continue


    def get_label_and_points(self, img):
        points = [np.array([50, 50]), np.array([200, 200])]
        labels = ["buoy", "totem"]
        return (points, labels)

    def get_ray_in_enu(self, point, camera, R):
        # find ray and then put into inu
        ray = np.array(camera.projectPixelTo3dRay(point))
        return R.dot(ray)
        # return enu_cam_tf.transform_vector(ray)
    def find_closest_object_given_ray(self, objects, ray, ray_base, tol = 3):
        for o in objects:
            distance = np.linalg.norm(np.cross(ray, mil_tools.rosmsg_to_numpy(o.pose.position) - ray_base))
            # print (distance)
            if distance > tol:
                continue
            else:
                return o
        return None
    def set_label_for_object(self, label, o):
        try:
            all_objects = self.DBQuery_proxy(name="", cmd="{}={}".format(o.id, label)).objects
            # print ("Set {} to {}".format(o.id, label))
        except rospy.ServiceException, e:
            print ("Couldn't set {} to {}".format(o.id, label))

if __name__ == "__main__":
    rospy.init_node("singleshot_detector")
    tf_listener = tf.TransformListener()

    big_boi = ObjectImageClassifier()

    rospy.spin()