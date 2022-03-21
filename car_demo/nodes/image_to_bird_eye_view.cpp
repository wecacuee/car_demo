#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <iostream>
#include <cstdlib>

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
eigen_imshow(const Eigen::MatrixXd& img,
             const std::string& winname = "IMG",
             const int duration = 10  // in milliseconds
    ) {
    // Eigen -> OpenCV
    cv::Mat cv_img;
    cv::eigen2cv(img, cv_img);

    cv::imshow(winname, cv_img);
    cv::waitKey(duration);
}

double drand() {
    return std::rand()  / double(RAND_MAX);
}

void chatterCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // ROS_INFO("Image height: [%d]", msg->height);
    // ROS_INFO("Image width: [%d]", msg->width);
    // ROS_INFO("Image encoding: [%d]", msg->width);
  Eigen::MatrixXd eigen_image;
  convert_image_msg_to_eigen(msg, eigen_image);

  eigen_imshow(eigen_image);

  Eigen::MatrixXd birds_eye_view_image(eigen_image.rows(),
                                       eigen_image.cols());
  birds_eye_view_image.setZero();

  double camera_height = 1.4; // meters
  Eigen::Matrix3d K;
  K << 476.7030836014194, 0.0, 400.5, 0.0, 476.7030836014194, 400.5, 0.0, 0.0, 1.0;
  Eigen::Matrix3d Kinv = K.inverse();
  Eigen::Matrix3d R;
  R << 1, 0, 0,
      0, 0, 1,
      0, -1, 0;
  Eigen::Vector3d t;
  t << 0, -10*camera_height, 10*camera_height;
  Eigen::MatrixXd KRKinv = K * R * Kinv;
  Eigen::Vector3d Kt =  K*t;
  Eigen::Vector3d u_bev;
  for (int bev_row = 0; bev_row < birds_eye_view_image.rows(); ++bev_row) {
      for (int bev_col = 0; bev_col < birds_eye_view_image.cols(); ++bev_col)  {
          u_bev << bev_col + 0.5, bev_row + 0.5, 1;
          // Eigen::Vector3d lambda_X_bev = Kinv * u_bev;
          // Eigen::Vector3d X_bev = lambda_X_bev / lambda_X_bev(2) * (-t(1)+camera_height);
          // Eigen::Vector3d X = R * X_bev + t;
          auto lambda_u = KRKinv * u_bev * (-t(1)+camera_height) / (Kinv.row(2)*u_bev) + Kt;
          Eigen::Vector3d u = lambda_u / lambda_u(2);
          int col = (int) u(0);
          int row = (int) u(1);
          if (0 <= row &&  row < eigen_image.rows()
              &&  0 <= col &&  col < eigen_image.cols()) {
              birds_eye_view_image(bev_row, bev_col) = eigen_image(row, col);
          }
      }
  }

  eigen_imshow(birds_eye_view_image, "BEV");

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
  ros::Subscriber sub = n.subscribe("/prius/front_camera/image_raw", 1, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
