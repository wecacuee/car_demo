#include "ros/ros.h"
#include "sensor_msgs/Image.h"

// sudo apt install libeigen3-dev ros-noetic-cv-bridge
// Additional header files  required  for conversion
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

/**
 * Convert ROS's sensor_msgs::Image::ConstPtr to Eigen's Eigen::MatrixXd
 *
 * Example usage: (if msg is of type sensor_msgs::Image::ConstPtr):

 *  Eigen::MatrixXd eigen_image;
 *  convert_image_msg_to_eigen(msg, eigen_image);

 */
void
convert_image_msg_to_eigen(const sensor_msgs::Image::ConstPtr& msg,
                           Eigen::MatrixXd& returned_eigen_image) {
    // http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
    cv_bridge::CvImageConstPtr cv_image_ptr;
    try {
        cv_image_ptr = cv_bridge::toCvShare(
            msg,
            sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception& e) {
        ROS_FATAL("cv_bridge exception: %s", e.what());
        return;
    }

    // Convert color image to gray scale
    cv::Mat gray_image;
    cv::cvtColor(cv_image_ptr->image, gray_image, cv::COLOR_RGB2GRAY);

    // Scale the image doubles between 0 and 1
    cv::Mat gray_image_double;
    gray_image.convertTo(gray_image_double, CV_64FC1);
    gray_image_double /= 255.0;

    // OpenCV -> Eigen
    cv::cv2eigen(gray_image_double,  returned_eigen_image);
}

/**
 * This function visualizes the Eigen image.
 *
 * Example  usage:
 *
 * eigen_imshow(eigen_image);
 *
 */
void
eigen_imshow(const Eigen::MatrixXd& img) {
    // Eigen -> OpenCV
    cv::Mat cv_img;
    cv::eigen2cv(img, cv_img);

    cv::imshow("IMG", cv_img);
    cv::waitKey(10);
}

void chatterCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO("Image height: [%d]", msg->height);
  ROS_INFO("Image width: [%d]", msg->width);
  ROS_INFO("Image encoding: [%s]", msg->encoding.c_str());

  Eigen::MatrixXd eigen_image;
  convert_image_msg_to_eigen(msg, eigen_image);

  eigen_imshow(eigen_image);
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/prius/front_camera/image_raw", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
