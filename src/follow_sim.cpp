#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/SpawnModel.h"
#include "ros/package.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <sstream>
#include <filesystem>

// https://learnopencv.com/object-tracking-using-opencv-cpp-python/
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include <numeric>
#include <limits>
#include <cmath>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <thread>

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

Mat rgb_img;
Mat depth_img;

Ptr<Tracker> tracker;
Rect2d roi;

Mat depth_info =  Mat::zeros(3, 3, CV_64FC1);;
Mat rgb_info =  Mat::zeros(3, 3, CV_64FC1);;

pcl::PointCloud<pcl::PointXYZ> cloud;

geometry_msgs::Twist movement;

// ros::Publisher mov_topic; // A topic which publishes the current movement.

std::string getFile(std::string filename)
{
  std::string buffer;
  char c;

  std::ifstream in(filename);
  if (!in)
  {
    std::cout << filename << " not found";
    exit(1);
  }
  while (in.get(c))
    buffer += c;
  in.close();

  return buffer;
}

// Subscribe to the depth image and rgb images.
// Then, Publish the movement commands to the teleop.

/*
    RGB topic: /camera/rgb/image_raw
    RGB msg type: sensor_msgs/Image

    Depth topic: /camera/depth/image_raw
    Depth msg type: sensor_msgs/Image

    https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy 
    http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
    http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
    http://docs.ros.org/en/jade/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html#abac29e84846ff8d139570348672795f6
*/
void rgbCallback(const sensor_msgs::ImageConstPtr& msg){
    // Constantly set the rgb image at chosen rate.
    // ROS_INFO_STREAM("RGB has arrived.");
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    // ROS_INFO_STREAM("New RGB from " << frame_id);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Verify in an open window.
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(3);
    // ROS_INFO_STREAM(cv_ptr->image);

    rgb_img = cv_ptr->image;
}

/*
    Point Cloud Topic: /camera/depth/points
    Point Cloud msg Type: sensor_msgs/PointCloud2
*/
void depthCallback(const sensor_msgs::PointCloud2ConstPtr & msg){
  // Get the depth point cloud. This is to be mapped to each pixel
  // in the rgb. 
  // ROS_INFO("Received PointCloud2 message with width: %d and height: %d", msg->width, msg->height);

  // Width and height should be identical to the RGB image.
  pcl::fromROSMsg(*msg, cloud);

  // ROS_INFO_STREAM(cloud.size()); // Length should be resolution height * width

}

/*
  Get the camera intrinsics matrix.
  Camera info topic: /camera/depth/camera_info & /camera/rgb/camera_info
  Camera info msg type: sensor_msgs/CameraInfo (interested in K matrix)

  https://stackoverflow.com/questions/48502215/how-to-assign-the-sensor-msgs-camerainfo-to-cvmat
*/
void depthIntrinsicsCallback(const sensor_msgs::CameraInfoConstPtr & msg){
    // Check if the K matrix has the correct size
    if (msg->K.size() == 9) {
        // Create a 3x3 matrix to store the K matrix
        cv::Mat K(3, 3, CV_64F);
        
        // Fill the matrix with values from the K array
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                K.at<double>(i, j) = msg->K[i * 3 + j];
            }
        }
        
        // ROS_INFO_STREAM("K matrix: " << K);
        depth_info = K;

        // Optionally, you can convert it to a more usable format or use it directly
    } else {
        ROS_WARN("Invalid K matrix size.");
    }
  // depth_info = msg->K.data();
}

void rgbIntrinsicsCallback(const sensor_msgs::CameraInfoConstPtr & msg){
    // Check if the K matrix has the correct size
    if (msg->K.size() == 9) {
        // Create a 3x3 matrix to store the K matrix
        cv::Mat K(3, 3, CV_64F);
        
        // Fill the matrix with values from the K array
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                K.at<double>(i, j) = msg->K[i * 3 + j];
            }
        }
        
        // ROS_INFO_STREAM("K matrix: " << K);
        rgb_info = K;
        
        // Optionally, you can convert it to a more usable format or use it directly
    } else {
        ROS_WARN("Invalid K matrix size.");
    }
  }

// Function to compute the average of valid PointXYZ points
pcl::PointXYZ computeAverage(const std::vector<pcl::PointXYZ>& validPoints) {
    pcl::PointXYZ average;
    int count = validPoints.size();

    if (count == 0) {
        return average; // Return a point with default values (0, 0, 0) if no valid points
    }

    for (const auto& point : validPoints) {
        average.x += point.x;
        average.y += point.y;
        average.z += point.z;
    }

    average.x /= count;
    average.y /= count;
    average.z /= count;

    return average;
}

/*
    Teleop topic: /cmd_vel
        /cmd_vel/angular/(x,y,z) & /cmd_vel/linear/(x,y,z)
    Teleop msg type: geometry_msgs/Twist
*/
geometry_msgs::Twist followCommand(){
/* 
    Given the current rgb and depth, detect the presence of a human (the closest?)
    If a human is detected, calculate their relative position in 3D space.
    Calculated the required angular and linear motion to move within the 
    desired range of that person.

    If no human is found within desired range, set all motion to zero.
*/
    geometry_msgs::Twist msg;
    // If images are not yet loaded, do nothing.
    if (!rgb_img.rows || !rgb_img.cols){
        return msg;
    }

    // https://docs.opencv.org/4.x/d2/d0a/tutorial_introduction_to_tracker.html 
    if (!roi.width || !roi.height){ // While no region of interest is selected.
      roi = selectROI("tracker", rgb_img);
      if(!roi.width || !roi.height){
        return msg;
      }
      else {
        tracker->init(rgb_img, roi); // Initialise 
        return msg; // Get tracking on next loop. Safe to delete?
      }
    }

    // Update the tracker with new frame.
    // ROS_INFO_STREAM(roi);
    bool trackSuccess = tracker->update(rgb_img, roi);
    if (trackSuccess == false){
      ROS_INFO_STREAM("Tracking subject lost!");
      cv::imshow(OPENCV_WINDOW, rgb_img);
      cv::waitKey(3);      
      return msg;
    }
    rectangle(rgb_img, roi, Scalar(255,0,0), 2, 1);

    // To calculate the 3D Position:
    // point = ((pixel position - principal points) * depth)/focal_length
    // (rgb_info.at<double>(0,0)); -> focal length x.
    // (rgb_info.at<double>(1,1)); -> focal length y
    // (rgb_info.at<double>(0,2)); -> principal point x
    // (rgb_info.at<double>(1,2)); -> principal point y

    // Get the centroid and limits of the roi bounding box, which is Rect2d.
    // It contains the top left corner coordinates, width, and height of the bbox.
    cv::Point2d topLeft(roi.x, roi.y);
    int width = roi.width;
    int height = roi.height;
    // cv::Point2d centroid(roi.x + roi.width/2, roi.y + roi.height/2);
    // https://answers.ros.org/question/238779/obtaining-point-by-using-column-row-coordinate/
    // ROS_INFO_STREAM(cloud.at(centroid.x,centroid.y));
    // x is left/right position
    // y is height
    // z is depth

    // Get the average depth within the roi bounding box.
    // Also look up the focal length of the lens? No longer needed with point cloud.
    // ROS_INFO_STREAM(cloud.height);
    std::vector<pcl::PointXYZ> validPoints;
    for (int i = topLeft.y; i < topLeft.y + height; i++){ // Move in y and height. 
      for (int j = topLeft.x; j < topLeft.x + width; j++){ // Move in x and width.
        // Check that x and y are not out of bounds.
        // ROS_INFO_STREAM(cloud.width);
        // ROS_INFO_STREAM(cloud.height);
        if((i < 0) || (j < 0) || (i >= cloud.height) || (j >= cloud.width)){
        // if((i < 0) || (j < 0) || ((j*i) > (cloud.height*cloud.width))){
  
          continue; // Do not attempt to get out of bounds depth data.
        }
        // ROS_INFO_STREAM(i);
        pcl::PointXYZ point3d = cloud.at(j, i); // If pointcloud is 2d.
        // pcl::PointXYZ point3d = cloud.at(j*i); // If pointcloud is 1d.
        if (std::isnan(point3d.x) || std::isnan(point3d.y) || std::isnan(point3d.z)){
          continue;
        }
        validPoints.push_back(point3d);
      }
    }

    if (validPoints.empty()){ // If there are no valid points for depth, return no movement.
      ROS_INFO_STREAM("No valid depth data!");
      cv::imshow(OPENCV_WINDOW, rgb_img);
      cv::waitKey(3);
      return msg;
    }

    pcl::PointXYZ average = computeAverage(validPoints);
    // ROS_INFO_STREAM(average);

    // Calculate XY position of the person with respect to robot.
    // If bbox is not centred, rotate.
    // If euclidean distance is too big, move forwards, if too small, move back.


    // Waffle max linear = 0.26
    // Waffle max angular = 1.82

    // If using simulations, turn the speeds down or it will overshoot.
    // Linear x to go back and forth. Depends on z.
    msg.linear.x = 0.0;
    if (average.z > 2.0){ // Go forwards - too far.
      msg.linear.x = 0.23;
    }
    else if(average.z < 1.5){ // Go backwards - too close.
      msg.linear.x = -0.23;
    }

    // Angular z to rotate. Depends on x.
    msg.angular.z = 0.0;
    if (average.x > 0.2){ // Person is to the right - turn left.
      msg.angular.z = -0.2;
    }
    else if(average.x < -0.2){
      msg.angular.z=0.2;
    }

    // Check that image Mat is accessible outside of callbacks.
    cv::imshow(OPENCV_WINDOW, rgb_img);
    cv::waitKey(3);
    ROS_INFO_STREAM(average.z);
    
    return msg;
}

void pub_loop(ros::NodeHandle n){
  ros::Publisher turtleMove = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  while (true){
    if (roi.width && roi.height){
      turtleMove.publish(movement);
    }
  }
}

int main(int argc, char **argv){

    // Set up person detection model.
    // model = dnn::readNetFromONNX("yolov4.onnx");
    // model.setPreferableBackend(dnn::DNN_BACKEND_OPENCV);
    // model.setPreferableTarget(dnn::DNN_TARGET_CPU);

    // Set up tracker.
    // https://learnopencv.com/object-tracking-using-opencv-cpp-python/
    std::string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};

    std::string trackerType = trackerTypes[7]; // Choose which tracker.

    if (trackerType == "BOOSTING")
      tracker = TrackerBoosting::create();
    if (trackerType == "MIL")
      tracker = TrackerMIL::create();
    if (trackerType == "KCF")
      tracker = TrackerKCF::create();
    if (trackerType == "TLD")
      tracker = TrackerTLD::create();
    if (trackerType == "MEDIANFLOW")
      tracker = TrackerMedianFlow::create();
    if (trackerType == "GOTURN")
      tracker = TrackerGOTURN::create();
    if (trackerType == "MOSSE")
      tracker = TrackerMOSSE::create();
    if (trackerType == "CSRT")
      tracker = TrackerCSRT::create();

    // ROS_INFO_STREAM('HERE!');

    ros::init(argc, argv, "follow_sim");

    ros::NodeHandle n;


    // Spawn a person.
    // Models: https://bitbucket.org/theconstructcore/person_sim/downloads/

    // These topics work with the simulation.

    // Subscribe to cameras in simulation.
    ros::Subscriber rgbSub = n.subscribe("/camera/rgb/image_raw", 1, rgbCallback);
    ros::Subscriber depthSub = n.subscribe("/camera/depth/points", 1, depthCallback);

    // Legacy topics. Not needed.
    // ros::Subscriber depthInfoSub = n.subscribe("/camera/depth/camera_info", 1, depthIntrinsicsCallback);
    // ros::Subscriber rgbInfoSub = n.subscribe("/camera/rgb/camera_info", 1, rgbIntrinsicsCallback);
    // ros::Subscriber rgbInfoSub = n.subscribe("/camera/color/camera_info", 1, rgbIntrinsicsCallback);

    // These topics work with the realsense camera connected to the laptop. roslaunch turtlebot3_bringup turtlebot3_robot.launch
    // ros::Subscriber rgbSub = n.subscribe("/camera/color/image_raw", 1, rgbCallback);
    // ros::Subscriber depthSub = n.subscribe("/camera/depth_registered/points", 1, depthCallback);

    // Make publisher for movement commands. Open turtlebot teleop for this.
    ros::Publisher turtleMove = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Rate loop_rate(100);

    // Uncomment if using simulation. For continuous movement.
    std::thread publish(pub_loop, n);

    while (ros::ok()){
        ros::spinOnce();

        // Comment out if using simulation.
        // turtleMove.publish(movement);

        // Calculate and publish movement command on the thread.
        movement = followCommand();

        // Comment out if using simulation. 
        // turtleMove.publish(movement);

        // ROS_INFO_STREAM("here");


        loop_rate.sleep();

    }

    return 0;

}