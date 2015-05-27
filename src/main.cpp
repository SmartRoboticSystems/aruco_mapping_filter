/*********************************************************************************************//**
* @file main.cpp
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

// ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

// My libraries
#include <filter.h>

////////////////////////////////////////////////////////////////////////////////

int
main(int argc, char **argv)
{
    //Bol som tu ... F.D. 
    
    // Inicialization of ROS
    ros::init(argc, argv, "image_filtering_for_aruco");
    ros::NodeHandle myRosNode;

    // Image transport node
    image_transport::ImageTransport it(myRosNode);

    // Get Paramter
    int myTreshold;
    myRosNode.getParam("treshold",myTreshold);

    // Filter object
    Filter myFilter(&it,myTreshold);

    // Subscriber for image
    image_transport::Subscriber subVideo;
    subVideo=it.subscribe("camera/image_raw",1,&Filter::ImageCallback,&myFilter);

    ros::spin();

	return 0;
}

////////////////////////////////////////////////////////////////////////////////
