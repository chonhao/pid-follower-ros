#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cstring>
#include <iostream>

static const std::string OPENCV_WINDOW = "Image window";
cv_bridge::CvImagePtr cv_ptr;

using namespace std;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/camera/depth/image_smoothed", 1);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::UMat depthf(cv_ptr->image.rows, cv_ptr->image.cols, CV_16UC1);
    cv_ptr->image.copyTo(depthf);
    const unsigned char noDepth = 0;
    cv::UMat temp, temp2, mask;
    cv::UMat small_depthf; 
    resize(depthf, small_depthf, depthf.size(), 0.2, 0.2);
    cv::compare(small_depthf, noDepth, mask, cv::CMP_EQ);
    cv::inpaint(small_depthf, mask, temp, 1.0, cv::INPAINT_TELEA);
    // resize(temp, temp2, depthf.size());

    // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(3);

    // Output modified video stream
    cv_bridge::CvImage img_bridge;
    img_bridge.header = msg->header;
    img_bridge.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    img_bridge.image = temp.getMat(cv::ACCESS_FAST);
    image_pub_.publish(img_bridge.toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  cv::waitKey(3);
  ros::spin();
  return 0;
}