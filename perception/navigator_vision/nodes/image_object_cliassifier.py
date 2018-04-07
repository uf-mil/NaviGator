import numpy as np
import rospy
import image_geometry
import mil_tools
from sensor_msgs.msg import CameraInfo, Image
import tf

class ImageGetter(object):
    def __init__(self, topic_name):
        self.sub = mil_tools.Image_Subscriber(topic_name, self.get_image)

        print 'getting topic', topic_name
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
        print ("cool")
        self.camera_topics = ["/camera/front/left/image_rect_color", "/camera/seecam/image_rect_color"]
        self.camera_tfs = ["/front_left_cam", "/seecam"]

        self.cameras = [ImageGetter(p) for p in self.camera_topics]

        while None in [p.img_geom for p in self.cameras] and not rospy.is_shutdown():
            print (self.cameras)
            print "Waiting frames..."
            rospy.sleep(1)

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

                for i in range(len(self.cameras)):
                    points, labels = info[i]
                    for j in range(len(points)):
                        ray = self.get_ray_in_enu(points[j], self.cameras, enu_cam_tf[i])

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

    def get_ray_in_enu(self, point, camera, enu_cam_tf):
        # find ray and then put into inu
        ray = np.array(camera.projectPixelTo3dRay((point)))
        enu_cam_tf.transform_vector(ray)



if __name__ == "__main__":
    rospy.init_node("singleshot_detector")
    tf_listener = tf.TransformListener()

    big_boi = ObjectImageClassifier()

    rospy.spin()