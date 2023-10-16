#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"

#define CAMS 1

using namespace cv;
using namespace std;

class Cam_pub : public rclcpp::Node{
private:
    Mat cameraMat, dist;
    rclcpp::TimerBase::SharedPtr timer_;
    image_transport::Publisher image_publisher_[CAMS];
    image_transport::Publisher image_undistorted_publisher_[CAMS];

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;

    size_t count_;
    VideoCapture cap[CAMS];
    cv_bridge::CvImagePtr cv_ptr;

public:
    Cam_pub() : Node("cam_pub"), count_(0)    {
        rclcpp::NodeOptions options;
        rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher", options);
        image_transport::ImageTransport it(node);


        for (int x = 0; x < CAMS; x++) {
            cap[x] = VideoCapture(x);

            image_publisher_[x] = it.advertise("image_" + to_string(x), 1);
            image_undistorted_publisher_[x] = it.advertise("image_undistorted_" + to_string(x), 1);
        }

        image_subscriber = this->create_subscription<sensor_msgs::msg::Image>
                ("/image", 10, [this](const sensor_msgs::msg::Image & image_msg){
                    timer_callback(image_msg);
                });

        //timer_ = this->create_wall_timer(16ms, std::bind(&Cam_pub::timer_callback, this));

        read_params();
    }

private:

    void read_params(){
        cv::Mat cameraMatrix(3, 3, CV_64F);
        cv::Mat distCoeffs(1, 5, CV_64F);

        std::ifstream inFile("./src/lidar_camera_early_fusion/calibration_data/intrinsic/intrinsic_test.txt");
        if (!inFile.is_open()) {
            std::cerr << "Error: Couldn't open the camera parameter file." << std::endl;
            return ;
        }

        std::string line;
        while (std::getline(inFile, line)) {
            if (line.find("K:") != std::string::npos) {
                std::istringstream valueStream(line.substr(2)); // Remove "K:"

                std::vector<double> values;
                double value;
                while (valueStream >> value) {
                    values.push_back(value);
                }

                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        cameraMatrix.at<double>(i, j) = values[i * 3 + j];
                    }
                }
            } else if (line.find("D:") != std::string::npos) {
                std::istringstream valueStream(line.substr(2)); // Remove "D:"
                double value;
                for (int i = 0; i < 5; i++) {
                    valueStream >> value;
                    distCoeffs.at<double>(0, i) = value;
                }
            }
        }

        // Output the parsed camera matrix and distortion coefficients
        cameraMat = cameraMatrix;
        dist = distCoeffs;
        std::cout << "Camera Matrix (K):" << std::endl << cameraMatrix << std::endl;
        std::cout << "Distortion Coefficients (D):" << std::endl << distCoeffs << std::endl;

        inFile.close();
    }

    void timer_callback (const sensor_msgs::msg::Image & image_msg){
        try{
            cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
        }
        catch (cv_bridge::Exception& e){
            std::cout << "cv_ptr == NULL" << std::endl;
            return;
        }

        for (int x = 0; x < CAMS; x++) {
            Mat image = cv_ptr->image;
            //cap[x] >> image;

            cv::Mat undistorted;
            cv::undistort(image, undistorted, cameraMat, dist);

            sensor_msgs::msg::Image::SharedPtr message =
                    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image)
                            .toImageMsg();

            sensor_msgs::msg::Image::SharedPtr undistorted_message =
                    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", undistorted)
                            .toImageMsg();

            message->header.stamp = this->get_clock()->now();
            message->header.frame_id = "webcam";
            undistorted_message->header = message->header;
            image_publisher_[x].publish(message);
            image_undistorted_publisher_[x].publish(undistorted_message);
        }
    }
};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Cam_pub>());
	rclcpp::shutdown();
	return 0;
}
