/*********************************************************************************************//**
* @file filter.h
*
* Image filter for sharping edges and removing reflection
*
* Copyright (c)
* Jan Bacik
* Smart Robotic Systems
* www.smartroboticsys.eu
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

#ifndef FILTER_H
#define FILTER_H

////////////////////////////////////////////////////////////////////////////////

// ROS libraries
#include <ros/ros.h>

// Image libraries
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

////////////////////////////////////////////////////////////////////////////////

class Filter
{
public:
    explicit Filter(image_transport::ImageTransport *node, int treshold);
    ~Filter();
    void ImageCallback(const sensor_msgs::ImageConstPtr &original_image);

private:
    // Treshold
    int myTreshold;
    // CV_bridge converts std_msgs::Image to OpenCV
    cv_bridge::CvImagePtr cv_ptr;
    // Mat structure
    cv::Mat I;
    cv::Mat I_filtered;
    // Publisher for filtered image
    image_transport::Publisher pubFilteredImage;

};

////////////////////////////////////////////////////////////////////////////////

#endif // FILTER_H
