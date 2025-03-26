#include <ros/ros.h>
#include "../include/antdroid_cam_control/antdroid_cam_control.hpp"

ImageConverter::ImageConverter(): _it(_nh),
                                  _count(0),
                                  _is_test(0)
{
    // Subscribe to the camera image topic
    _image_sub = _it.subscribe("/camera/image", 1, 
    &ImageConverter::imageCb, this);

    // Advertise to the robot control topic
    _vel_pub = _nh.advertise<geometry_msgs::Twist>("/control_interpreter/cmd_vel", 1, true);

    // Get the test parameter for displaying the image
    _nh.getParam("/cam_control/test", _is_test);
    if (_is_test)
    {
        namedWindow("Brightness Detection"); // Open a window for testing the brightness detection
    }
}

ImageConverter::~ImageConverter()
{
    if (_is_test)
    {
        destroyWindow("Brightness Detection"); // Close the window if testing
    }
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        // Convert the image from ROS message to OpenCV format
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    assert(cv_ptr->image.type() == CV_8UC3);

    // Convert to grayscale for brightness detection
    cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2GRAY);
    
    // Track the brightest area in the grayscale image
    trackLight(cv_ptr->image);
    
    waitKey(200);  // Add a small delay to avoid high CPU usage
}

void ImageConverter::trackLight(Mat src)
{
    // Threshold to isolate bright areas (adjust threshold as necessary)
    Mat thresholded;
    threshold(src, thresholded, 180, 255, THRESH_BINARY);

    // If testing, display the thresholded image
    if (_is_test)
    {
        imshow("Brightness Detection", thresholded);
    }

    // Find the brightest area (location of max intensity)
    Point maxLoc;
    double minVal, maxVal;
    minMaxLoc(src, &minVal, &maxVal, 0, &maxLoc);

    ROS_INFO_STREAM("Max Brightness at: " << maxLoc.x << ", " << maxLoc.y);

    // Control the robot to move towards the brightest area
    geometry_msgs::Twist vel;
    if (maxLoc.x < LEFT_WIDTH_ROTATE)
        vel.angular.z = 1;  // Rotate left
    else if (maxLoc.x > RIGHT_WIDTH_ROTATE)
        vel.angular.z = -1; // Rotate right
    else
        vel.linear.x = 1;   // Move forward

    // Publish the velocity command to control the robot
    if (vel.linear.x != 0 || vel.angular.z != 0)
        _vel_pub.publish(vel);
}
