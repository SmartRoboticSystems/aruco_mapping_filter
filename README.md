# aruco_mapping_filter

[ROS](http://ros.org) package

Image filter improving performance of [aruco_mapping](http://wiki.ros.org/aruco_mapping) package

* Author: [Jan Bacik] (http://www.smartroboticsys.eu/?page_id=895&lang=en), [Smart Robotic Systems] (http://www.smartroboticsys.eu)

<a href="http://www.youtube.com/watch?feature=player_embedded&v=fgW7b1jf4R8
" target="_blank"><img src="http://img.youtube.com/vi/fgW7b1jf4R8/0.jpg" 
alt="Aruco mapping filter" width="480" height="360" border="10" /></a>


## ROS API:

**Subsrcibed Topics**: */camera/image_raw*

**Published Topics**: */camera/image_raw_filtered*

**Parameters**:

Name          | Type         | Range       | Comment                  |
------------- | -------------| --------------------| -------------------------|
threshold | int | 0-255 | 0 means original image |


