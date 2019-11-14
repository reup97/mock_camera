/**
 * file mock_camera_node.cpp
 * author     Paul Liu
 * date       Nov 21 2016
 * brief      A mock camera used for publishing videos to some topic
 */

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>


/**
 * check if a given string is a number
 * @param  s reference to the string need to be checked
 * @return   true if s is a number, false otherwise
 */
bool isNumber(const std::string& s);

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "mock_camera");

    ros::NodeHandle nh;
    cv::VideoCapture cap;                                                                                                                                                                                                                                                                                                      
    std::string topicName;
    std::string videoFilePath;
	
    // get parameter topic
    if (nh.hasParam("topic"))
    {
    	nh.getParam("topic", topicName);

    }
    else
    {
    	ROS_FATAL("Does not have topic param. Terminating!");
        ros::shutdown();
    }

    // get parameter file_path
    if (nh.hasParam("file_path"))
    {
    	//NOTE: the parameter should be of type string,
    	//if no param specified in launch file, open defaault camera
    	nh.getParam("file_path", videoFilePath);
        if ( videoFilePath.empty() )
        {
            // not pass in any value. set to default
            videoFilePath = "0";
        }
    }
    else
    {
    	ROS_FATAL("Does not have file_path param. Terminating!");
        ros::shutdown();
    }

    try
    {

    	if (isNumber( videoFilePath ))
    	{
    		cap.open(atoi(videoFilePath.c_str())); 
    	}
    	else if ( !videoFilePath.empty())
    	{
    		cap.open(videoFilePath);
    	}
    	else
    	{
    		ROS_FATAL_STREAM( "file path not found. Terminating!" );
            ros::shutdown();
    	}
    }
   	catch ( std::exception& e )
   	{
   		ROS_FATAL_STREAM( videoFilePath << ": " << e.what() );
        ros::shutdown();
   	}

   	image_transport::ImageTransport it(nh);
   	// advertise to specified topic
   	image_transport::Publisher pub = it.advertise(topicName, 1);

   	ROS_INFO("publish to : %s",topicName.c_str());
   	ROS_INFO("video path: %s", videoFilePath.c_str() );
    // camera not open correctly, return -1
    if ( !cap.isOpened() )
        return -1;


    ros::Rate loop_rate(30);
    for (;;)
    {
 
        cv::Mat frame;
        cap >> frame;
        // public to specified topic
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
        if( cv::waitKey (30) >= 0) break;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


bool isNumber(const std::string& s)
{
    std::string::const_iterator it = s.begin();
    while (std::isdigit(*it) && it != s.end() ) 
    	++it;
    return !s.empty() && it == s.end();
}
