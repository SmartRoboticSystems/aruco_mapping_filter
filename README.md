# image_filtering_for_aruco

[ROS](http://ros.org) package

Image filter for better performance of ArUco detector

* Video: https://www.youtube.com/watch?v=fgW7b1jf4R8
* ROS wiki page: 
* Autors: [Jan Bacik] (http://www.smartroboticsys.eu/?page_id=895&lang=en), [Smart Robotic Systems] (http://www.smartroboticsys.eu)


## Topics:

Input Image: /camera/image_raw

Output Image: /camera/image_raw_filtered

## Parameters:

Name          | Type         | Value       | Comment                  |
------------- | -------------| ------------| -------------------------|
treshold      | int          | 0-255       |  0 means original image  | 
