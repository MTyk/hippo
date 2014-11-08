
import rospy
import sys
import cv2
import cv2.cv as cv
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, RegionOfInterest
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
from math import cos

class hippoVision():
    def __init__(self):
        self.node_name = "hippo_vision"
        rospy.init_node(self.node_name)

        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)
        
        # Rospy loop rate 10Hz
        #self.rate = rospy.rate(10)
        
        # Create the OpenCV display window for the RGB image
        self.cv_window_name = self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        cv.MoveWindow(self.cv_window_name, 25, 75)
        
        # And one for the depth image
        cv.NamedWindow("Depth Image", cv.CV_WINDOW_NORMAL)
        cv.MoveWindow("Depth Image", 25, 350)
        
        # Create the cv_bridge object
        self.bridge = CvBridge()
        
        self.image_height = 0
        self.image_width = 0
        self.center = (0,0)
        
        self.depth_image_metric = None
        self.roi = (None, 0.0)
        
        self.default_camera_pose = (0, 1.57, 1.57)
        self.desired_camera_pose = (0, 1.57, 1.57)
        
        rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo)
        self.info_sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.get_camera_info)
        
        # Subscribe to the camera image and depth topics and set<
        # the appropriate callbacks
        rospy.wait_for_message('/camera/depth/image_raw', Image)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        rospy.wait_for_message('/camera/rgb/image_color', Image)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.image_callback)
        
        self.roi_pub = rospy.Publisher('/roi', RegionOfInterest, queue_size=1)
        self.pose_err_pub = rospy.Publisher('/pose_error', Twist, queue_size=3)
        
        rospy.loginfo("Waiting for image topics...")
        
        
    def get_camera_info(self, msg):
        # Init image resolution based on first msg
        if (self.center[0] == 0) and (self.center[1] == 0):
            self.image_height = msg.height
            self.image_width = msg.width
            self.center = (self.image_width/2, self.image_height/2)
            rospy.loginfo("Found camera with a resolution of %d x %d", self.image_width, self.image_height)
        
        
    def image_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        
        if self.depth_image_metric != None:
        
            # Convert the image to a Numpy array since most cv2 functions
            # require Numpy arrays.
            frame = np.array(frame, dtype=np.uint8)
            
            # Process the frame using the process_image() function
            display_image = self.process_image(frame)
            
            # Find closest region of interest
            self.roi = self.find_closest_roi(display_image, frame)
            self.mark_roi(frame)
            
            # Display the image.
            cv2.imshow(self.node_name, frame)
            
            pixel_offset = self.calculate_pixel_offset()
            rospy.loginfo("ROI offset: %s", str(pixel_offset))
            angle_offset = self.calculate_angle_offset(pixel_offset)
            
            if self.roi[0] != None:
                x,y,w,h = cv2.boundingRect(self.roi[0])
                roi_new = RegionOfInterest()
                roi_new.x_offset = x
                roi_new.y_offset = y
                roi_new.height = h
                roi_new.width = w
                roi_new.do_rectify = False
                self.roi_pub.publish(roi_new)
            
            # Process any keyboard commands
            self.keystroke = cv.WaitKey(5)
            if 32 <= self.keystroke and self.keystroke < 128:
                cc = chr(self.keystroke).lower()
                if cc == 'q':
                    # The user has press the q key, so exit
                    rospy.signal_shutdown("User hit q key to quit.")
        
        
    def depth_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # The depth image is a single-channel float32 image
            depth_image = self.bridge.imgmsg_to_cv(ros_image, "32FC1")
        except CvBridgeError, e:
            print e
        
        # Convert the depth image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        depth_array = np.array(depth_image, dtype=np.float32)
        self.depth_image_metric = np.copy(depth_array)
        
        # Normalize the depth image to fall between 0 and 1
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        
        # Process the depth image
        depth_display_image = self.process_depth_image(depth_array)
        
        # Display the result
        cv2.imshow("Depth Image", depth_display_image)
        
        
    def process_image(self, frame):

        '''
        # Create a black image, a window
        cv2.namedWindow('1. Detected edges')
        cv2.namedWindow('2. After noise filtering')
        cv2.namedWindow('3. After blob filtering')
        '''
        
        '''
        # create trackbars for threshold change
        cv2.createTrackbar('Upper threshold','1. Detected edges',0,255,self.nothing)
        cv2.createTrackbar('Lower threshold','1. Detected edges',0,255,self.nothing)
        cv2.createTrackbar('Filter threshold','2. After noise filtering',0,10000,self.nothing)
        cv2.createTrackbar('Blob threshold','3. After blob filtering',0,65535,self.nothing)
        '''
        # Convert image to grayscale
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blob_8U = frame
        
        '''
        while(1):
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
        '''
   
        upper = 200
        lower = 180
        f_thres = 7000
        b_thres = 30000
        
        '''
        # Get current positions of trackbars
        upper = cv2.getTrackbarPos('Upper threshold','1. Detected edges')
        lower = cv2.getTrackbarPos('Lower threshold','1. Detected edges')
        f_thres = cv2.getTrackbarPos('Filter threshold','2. After noise filtering')
        b_thres = cv2.getTrackbarPos('Blob threshold','3. After blob filtering')
        '''
        
        edges_8U = cv2.Canny(frame, lower, upper)
        #cv2.imshow('1. Detected edges', edges_8U)
        
        edges_16U = edges_8U.astype(np.uint16)
        
        filtered_16U = cv2.boxFilter(edges_16U, -1, (15,15), -1, (-1, -1), False)
        
        low_values_indices = filtered_16U < f_thres # Where values are low
        filtered_16U[low_values_indices] = 0
        #high_values_indices = filtered_16U > f_thres # Where values are high
        #filtered_16U[high_values_indices] = 255
        cv2.normalize(filtered_16U, filtered_16U, 0, 255, cv2.NORM_MINMAX, cv2.CV_16UC1)
        
        filtered_8U = filtered_16U.astype(np.uint8)

        #cv2.imshow('2. After noise filtering', filtered_8U)
        
        blob_16U = cv2.boxFilter(filtered_16U, -1, (30,30), -1, (-1, -1), False)
        low_values_indices = blob_16U < b_thres # Where values are low
        blob_16U[low_values_indices] = 0
        high_values_indices = blob_16U > b_thres # Where values are high
        blob_16U[high_values_indices] = 255
        
        blob_8U = blob_16U.astype(np.uint8)

        #cv2.imshow('3. After blob filtering', blob_8U)
            
        #cv2.destroyAllWindows()
        return blob_8U
        
        
    def process_depth_image(self, frame):
        # Just return the raw image for this demo
        return frame
    
    
    def nothing(self, x):
        pass
    
    
    def find_closest_roi(self, blob_frame, color_frame):
        contours, hierarchy = cv2.findContours(blob_frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
        min_area = 3000
        max_area = 200000
        box_ratio = 2
        closest = (None, 0.0)
    
        for cnt in contours:
            x,y,w,h = cv2.boundingRect(cnt)
            area = w * h
            if (min_area < area < max_area):     # filter by area (we don't want the small objects a.k.a. crap)
                if(w/h < box_ratio) and (h/w < box_ratio):    # filter by shape (mainly for filtering out wall-floor intersections)
                    
                    distance = self.depth_image_metric[y+(h/2)][x+(w/2)]
                    
                    px_off = 1
                    while (distance == 0.0) or (px_off < 51):
                        if (x+(w/2) + px_off) < self.image_width:
                            distance = self.depth_image_metric[y+(h/2)][x+(w/2) + px_off]
                            if (distance == 0.0):
                                if (x+(w/2) - px_off) >= 0:
                                    distance = self.depth_image_metric[y+(h/2)][x+(w/2) - px_off]
                        px_off += 1
                    
                    if (self.roi[0] == None) or (closest[1] == 0.0):
                        closest = (cnt, distance)
                    else:
                        if (distance < closest[1]):
                            closest = (cnt, distance)
                            
                    cv2.rectangle(color_frame, (x,y), (x+w, y+h), (0, 255, 0), 2)
                    rospy.loginfo("Object found at a distance of " + str(distance) + "mm.")
                    
        if closest[0] == None:
            rospy.loginfo("Robot did not find any ROIs.")
        else:
            rospy.loginfo("Robot found a ROI!")
            rospy.loginfo("Distance to object is " + str(closest[1]) + " mm.")
            
        return closest
    
    
    def mark_roi(self, color_frame):
        if self.roi[0] != None:
            x,y,w,h = cv2.boundingRect(self.roi[0])
            cv2.rectangle(color_frame, (x,y), (x+w, y+h), (255, 0, 0), 2)
            
    
    def calculate_pixel_offset(self):
        if self.roi[0] != None:
            x,y,w,h = cv2.boundingRect(self.roi[0])
            roi_center = (x+(w/2), y+(h/2))
            x_offset = roi_center[0] - self.center[0]
            y_offset = roi_center[1] - self.center[1]
            return (x_offset, y_offset)
        return (0,0)
    
    
    def calculate_angle_offset(self, offset):
        if (self.roi[0] != None) and (self.roi[1] > 0.0):
            y_gain = 0.01
            tolerance = 0.1
            x,y,w,h = cv2.boundingRect(self.roi[0])
            distance_to_center = self.depth_image_metric[y+(h/2)][self.image_width/2]
            angle_offset_z = cos(distance_to_center/self.roi[1])
            angle_offset_y = y_gain * offset[1]
            if abs(angle_offset_z) < tolerance:
                rospy.loginfo("Z angle offset is below tolerance")
                angle_offset_z = 0
            if abs(angle_offset_y) < tolerance:
                rospy.loginfo("Y angle offset is below tolerance")
                angle_offset_y = 0
            return (angle_offset_z, angle_offset_y)
        
        
    def move_camera(self, pixel_offset, angle_offset):
        if self.roi == None:
            # move camera to default pose
            pass
        elif (angle_offset[0] == 0) and (angle_offset[1] == 0):
            rospy.logdebug("Camera is looking at object.")
            rospy.logdebug("Aligning robot with object.")
            self.align_robot()
        else:
            if pixel_offset[0] < 0:
                angle_offset[0] = -angle_offset[0]
            if pixel_offset[1] >= 0:
                angle_offset[1] = -angle_offset[1]
            x, y, z = self.default_camera_pose
            self.desired_camera_pose = (x, y + angle_offset[1], z + angle_offset[0])
            # publish to camera control node
            
    
    def align_robot(self):
        pose_error = Twist()
        pose_error.linear.x = self.roi[1]
        pose_error.linear.z = 0.5
        pose_error.angular.x = self.desired_camera_pose[2] - self.default_camera_pose[2]
        pose_error.angular.z = 0.1
        self.pose_err_pub.publish(pose_error)
    
    
    def mask_red(self, color_frame):
        hsv = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)
    
        # define rcange of blue color in HSV
        lower_red = np.array([170,100,100])
        upper_red = np.array([190,255,255])
    
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        return mask
            
        
    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()       

    
if __name__ == '__main__':
    try:
        rospy.init_node("hippo_vision")
        hippo = hippoVision()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()   
    
    
