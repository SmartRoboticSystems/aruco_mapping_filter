/*********************************************************************************************//**
* @file filter.cpp
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

#ifndef FILTER_CPP
#define FILTER_CPP
#endif

////////////////////////////////////////////////////////////////////////////////

#include "filter.h"

////////////////////////////////////////////////////////////////////////////////

Filter::Filter(image_transport::ImageTransport *node, int treshold)
{
    // Controlling of value of threshold
    if(treshold>255)
        myTreshold=0;
    else
        myTreshold=treshold;

    // Publisher fitered image
    pubFilteredImage=node->advertise("camera/image_raw_filtered",1);
}

////////////////////////////////////////////////////////////////////////////////

Filter::~Filter()
{

}

////////////////////////////////////////////////////////////////////////////////

void
Filter::ImageCallback(const sensor_msgs::ImageConstPtr &original_image)
{
    // Format
    cv_ptr=cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::MONO8);
    // OpenCV to MAT structure
    I=cv_ptr->image;

    // Sharpen filter Gaussian Blur
    cv::GaussianBlur(I, I_filtered, cv::Size(0,0),2);
    // Weights
    cv::addWeighted(I, 2.5, I_filtered, -1.5, 0, I_filtered);
    // Equalize histogram
    cv::equalizeHist(I_filtered,I_filtered);

    // Treshold
    cv::threshold(I_filtered,I_filtered,myTreshold,0,3);

    // Creating new filtered image_raw and publishing
    cv_bridge::CvImagePtr in_msg;
    in_msg=cv_ptr;
    cv_bridge::CvImage out_msg;
    // Same timestamp and tf frame as input image
    out_msg.header=in_msg->header;
    // Format
    out_msg.encoding=sensor_msgs::image_encodings::MONO8;
    out_msg.image=I_filtered;

    // Publishing
    pubFilteredImage.publish(out_msg.toImageMsg());
}

////////////////////////////////////////////////////////////////////////////////
