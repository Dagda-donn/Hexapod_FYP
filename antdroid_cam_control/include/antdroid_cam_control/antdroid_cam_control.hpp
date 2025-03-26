/* antdroid_cam_control.hpp: header of node to control antdroid with visual stimulus
 *
 * Copyright (C) 2015 Alexander Gil and Javier Román
 *
 * This library is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

 #ifndef ANTDROID_CAM_CONTROL_HPP_
 #define ANTDROID_CAM_CONTROL_HPP_
 
 #include <ros/ros.h>
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <opencv2/core/core.hpp>
 #include <geometry_msgs/Twist.h>
 
 static const int LEFT_WIDTH_ROTATE = 140;
 static const int RIGHT_WIDTH_ROTATE = 500;
 
 using namespace cv;
 
 class ImageConverter
 {
     ros::NodeHandle _nh;
     image_transport::ImageTransport _it;
     image_transport::Subscriber _image_sub;
     ros::Publisher _vel_pub;
     bool _is_test;
 
     int _count;
 
 public:
     ImageConverter();
     ~ImageConverter();
 
     void imageCb(const sensor_msgs::ImageConstPtr& msg);
 
     // Light tracking methods
     void trackLight(Mat src); // Track the brightest area and move towards it
 };
 
 #endif
 
