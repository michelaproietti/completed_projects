#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <stdlib.h>

void imageCallback(const sensor_msgs::CompressedImage& msg) {
	//converting from ROS image to OpenCV Mat
  	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	cv::Mat img = cv_ptr->image;

	//conversion to grayscale
	cv::Mat img_gray;
	cv::cvtColor(img, img_gray, CV_BGR2GRAY);

	//apply a threshold to clean the image
	double low_threshold = 127;
	double high_threshold = 255;
	cv::threshold(img_gray, img_gray, low_threshold, high_threshold, CV_THRESH_BINARY);

	//apply the gaussian blur
	cv::Mat img_blurred;
	cv::GaussianBlur(img_gray, img_blurred, cv::Size(3,3), 0.0, 0.0);
	
	//apply canny edge detection
	cv::Mat img_canny;
	cv::Canny(img_blurred, img_canny, low_threshold, high_threshold, 3);

	//show the original image and the new one
	const char *wnd_old = "Original image", *wnd_new = "Resulting image";
	cv::imshow(wnd_old, img);
	cv::imshow(wnd_new, img_canny);
	
	cv::waitKey(10);
}

int main (int argc, char** argv) {
	ros::init(argc, argv, "edge_extractor_node");
	ros::NodeHandle nh;

	ros::Subscriber image_sub = nh.subscribe("/default/camera_node/image/compressed", 1000, imageCallback);

	ros::spin();

	return 0;
}
