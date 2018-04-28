import os
import cv2
import numpy as np
import sys
import rospy
import tensorflow
from cv_bridge import CvBridge, CvBridgeError
import image_geometry
import mil_tools
from sensor_msgs.msg import CameraInfo, Image
import tf
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryResponse
from mil_msgs.msg import PerceptionObject

sys.path.append("..")

from cvod_utils import label_map_util
from cvod_utils import visualization_utils as vis_util


class CVOD(object):
    '''
    The Computer Vision based Object Detection is a trained neural network designed to detect buoys and totems in real time using cameras mounted on Navigator.
    By subscribing to the camera outputs through ROS, we are able to scan the video feeds for objects, labelling them and giving the coordinates of the objects relative to the boat.
    This is used in conjunction with the LIDAR to construct and occupancy grid that allows Navigator to autonomously avoid obstacles in real time.

    The network was trained using the Tensorflow library and through the process of transfer learning. A pretrained COCO model was loaded into the trainer, with most of the layers 'frozen'.
    These layers would not be changed during training. The last few layers of the network were then trained using our dataset of 400 images.

    @TODO:

    - Create a map to store ID from database and associated confidence level for each object. If new confidence is greater than or equal to previous confidence, update
    - Find a way to weight updates to Ogrid, not just labels.
    '''

    def __init__(self):
        rospy.init_node('active_detection')
        self.tf_listener = tf.TransformListener()
        ################ Init Database Services ###############################

        rospy.wait_for_service('/database/requests')
        self.DBQuery_proxy = rospy.ServiceProxy('/database/requests', ObjectDBQuery)

        ################ Init Publishers and Bridge ###########################
        # Image publisher for debugging, publishes images with bounding boxes
        # and labels drawn around detected objects.

        # Debug image for verification
        self.image_pub = rospy.Publisher(
            "/cvod/debug_images", Image, queue_size=1)

        '''
		CVBridge allows us to convert openCV images into ROS msgs and vice versa.
		It is the active link between publishers, subscribers and the operations performed between.
        '''
        self.bridge = CvBridge()

        ################ Init Subscribers #####################################
        # Used to keep track of when we should update PCODAR.
        self.see_frame_counter = 0
        self.left_frame_counter = 0

        # Image subscriber for seecam, these images will be processed by the
        # network.
        self.see_sub = mil_tools.Image_Subscriber(
            topic="/camera/seecam/image_raw", callback=self.callback, image_callback_args="/seecam")

        # Image subscriber for front left camera, these images will be
        # processed by the network.
        self.left_sub = mil_tools.Image_Subscriber(
            topic="/camera/front/left/image_raw", callback=self.callback, image_callback_args="/front_left_cam")

        ################ Get Camera Info ######################################

        			#### Seecam ####
        self.see_camera_info = self.see_sub.wait_for_camera_info()
        self.see_img_geom = image_geometry.PinholeCameraModel()
        self.see_img_geom.fromCameraInfo(self.see_camera_info)

   					#### Front Left ####
        self.left_camera_info = self.left_sub.wait_for_camera_info()
        self.left_img_geom = image_geometry.PinholeCameraModel()
        self.left_img_geom.fromCameraInfo(self.left_camera_info)

		###########################################################################

    def callback(self, cv_image, camera):
        '''
        A ROS callback is called everytime the subscriber gets a message.
        In our case, the message is converted to a CV_Image and then run through our pretrained network.
        These results are published for use in the PCODAR which populates our OGRID.
        We also send these results to a visualizer which returns an image with bounding boxes drawn and labelled for debugging.
        '''

        # 'Expand' the image to a standard size to be fed into algorithm.
        image_expanded = np.expand_dims(cv_image, axis=0)

        # Perform the actual detection by running the model with the image as
        # input
        (boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: image_expanded})
        # Draw the results of the detection (aka 'visualize the results')
        vis_util.visualize_boxes_and_labels_on_image_array(
            cv_image,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=8,
            min_score_thresh=0.10)

        image = cv2.resize(cv_image, (720, 480))
        # Publish our debug.
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))
        except CvBridgeError as e:
            print(e)

        # Figure out what camera we are looking at.
        if(camera == "seecam"):
        	# Tick up the counter.
            self.see_frame_counter = self.see_frame_counter + 1
            if(self.see_frame_counter >= 2):
	        	# After a minimum number of frames, update PCODAR. This should occur
	        	# roughly one time every second.
	        	self.update_pcodar(boxes, classes, scores, camera)
	        	self.see_frame_counter = 0

	    # We are looking at the left cam.
        else:
        	# Tick up counter.
        	self.left_frame_counter = self.left_frame_counter + 1
        	if(self.left_frame_counter >= 2):
	        	# After a minimum number of frames, update PCODAR. This should occur
	        	# roughly one time every second.
	        	self.update_pcodar(boxes, classes, scores, camera)
	        	self.left_frame_counter = 0

        return (boxes, classes)

    def update_pcodar(self, boxes, classes, scores, camera_name):
    	# Determine exactly where the image came from and retrieve the necessary
    	# information.
    	if camera_name == "/seecam":
    		camera = self.see_sub
    		img_geom = self.see_img_geom
        else:
    		camera = self.left_sub
    		img_geom = self.left_img_geom

    	# Get the transformation to allow us to get usable info from camera
    	# pixels.
        time = camera.last_image_time
        self.tf_listener.waitForTransform("/enu", camera_name, rospy.Time(0), rospy.Duration(10))

        enu_cam_tf = self.tf_listener.lookupTransform("/enu", camera_name, rospy.Time(0))

        if not enu_cam_tf:
            rospy.logwarn("No image recieved yet, are you sure the camera is publishing the right topic?")
            return
        cam_p, cam_q = enu_cam_tf
        cam_p = np.array(cam_p)
        cam_r = tf.transformations.quaternion_matrix(cam_q)[:3, :3]
        # Fetch the current objects on the Ogrid.
        all_objects = None
        try:
            all_objects = self.DBQuery_proxy(name="all", cmd="").objects
        except rospy.ServiceException, e:
            print ("Couldn't call database")
            rospy.sleep(5)
            return
        if all_objects is None:
            print ("Didn't find any objects")
            rospy.sleep(5)
            return
        # Attempt to label the objects we have found.
        points, labels = boxes, classes
        for j in range(len(points)):
            if scores[0][j] < 0.80:
                continue 
            for k in range(len(points[j])):
                ray = self.get_ray_in_enu(points[j][k], img_geom, cam_r)
            if ray is None:
                continue
            o = self.find_closest_object_given_ray(all_objects, ray, cam_p)
            if o is not None:
                self.set_label_for_object(labels[0][j], o)
            rospy.sleep(2)

    def get_ray_in_enu(self, point, camera, R):
        # find ray and then put into inu
        ray = np.array(camera.projectPixelTo3dRay(point))
        return R.dot(ray)

    def find_closest_object_given_ray(self, objects, ray, ray_base, tol = 3):
        # Compare a given bounding box with objects found by LIDAR. Find the closest object if any.
        for o in objects:
            distance = np.linalg.norm(np.cross(ray, mil_tools.rosmsg_to_numpy(o.pose.position) - ray_base))
            if distance > tol:
                continue
            else:
                return o
        return None

    def set_label_for_object(self, label, o):
        # Try and set the label once its found.
        try:
            if label > 1:
                label = "Totem"
            else:
                label = "Buoy"
            all_objects = self.DBQuery_proxy(name="", cmd="{}={}".format(o.id, label)).objects
        except rospy.ServiceException, e:
            print ("Couldn't set {} to {}".format(o.id, label))

if __name__ == '__main__':
    # Current working directory
    CWD_PATH = os.getcwd()

    # Path to our checkpoint, which contains a checkpoint file containing further information for constructing network.
    PATH_TO_CKPT = os.path.join(CWD_PATH, 'frozen_inference_graph.pb')

    # Labels used for identification.
    PATH_TO_LABELS = os.path.join(CWD_PATH, 'pcodar.pbtxt')

    # Number of Classes
    NUM_CLASSES = 6

    # Load lable map
    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
    category_index = label_map_util.create_category_index(categories)

    # Loading pretrained weights graph
    detection_graph = tensorflow.Graph()
    with detection_graph.as_default():
        od_graph_def = tensorflow.GraphDef()
        with tensorflow.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tensorflow.import_graph_def(od_graph_def, name='')

        sess = tensorflow.Session(graph=detection_graph)

    # Input tensor is the image
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

    # Output tensors are the detection boxes, scores, and classes
    # Each box represents a part of the image where a particular object was detected
    detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

    # Each score represents level of confidence for each of the objects.
    # The score is shown on the result image, together with the class label.
    detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
    detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

    # Number of objects detected
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')
    # Run the script
    CVOD()
    rospy.spin()
