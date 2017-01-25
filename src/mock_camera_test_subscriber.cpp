#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

static const std::string OPENCV_WINDOW = "Mock camera test subscriber";

void imageCallback( const sensor_msgs::ImageConstPtr& msg )
{
	try
	{
		cv::imshow(OPENCV_WINDOW, cv_bridge::toCvShare(msg, "bgr8")->image);
    	cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "mock_camera_test_subscriber");
	ros::NodeHandle nh;

	std::string topicName;

	if (nh.hasParam("topic"))
	{
		nh.getParam("topic", topicName );
	}
	else 
	{
		// try to subscript to "camera/image_raw"
		ROS_INFO("Topic name not specified, subscribe to `camera/image_raw`");
		topicName = "camera/image_raw";
	}


	cv::namedWindow(OPENCV_WINDOW);
	cv::startWindowThread();
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe(topicName, 1, imageCallback);
	ros::spin();
	cv::destroyWindow(OPENCV_WINDOW);
	return 0;
}