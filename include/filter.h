/*********************************************************************************************//**
* @file filter.h
*
* Copyright (c)
* SmartRoboticSystems
* March 2015
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/* Author: Jan Bacik */

#ifndef FILTER_H
#define FILTER_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

/** \brief Image filtering for Aruco */
namespace image_filtering_for_aruco
{
  
/** \brief Client class for Aruco image filtering */
class Filter
{
public:
  
  /** \brief Construct a client for EZN64 USB control*/    
  Filter(ros::NodeHandle* nh, image_transport::ImageTransport *node);
  
  ~Filter();
    
  /** \brief Callback function for filtering */    
  void ImageCallback(const sensor_msgs::ImageConstPtr &original_image);

private:
     /** \brief User threshold loaded from the launch file */
    int user_threshold;
    
    /** \brief CV bridge to convert std_msgs::Image to OpenCV format*/
    cv_bridge::CvImagePtr cv_ptr;
   
    /** \brief Original image */
    cv::Mat I;
    
    /** \brief Filtered image */
    cv::Mat I_filtered;
    
    /** \brief Filtered image publisher */
    image_transport::Publisher filtered_image_pub;

}; //Filter
}  //image_filtering_for_aruco

#endif // FILTER_H
