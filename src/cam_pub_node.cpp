#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace cv;
using namespace std;

class Cam_pub : public rclcpp::Node{
  public:
    Cam_pub() : Node("cam_pub"), count_(0)    {
    	sequence = 0;
	publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image", 10);
	timer_ = this->create_wall_timer(
	16ms, std::bind(&Cam_pub::timer_callback, this));
	cap = VideoCapture(0);
    }

  private:
    void timer_callback()    {
	cap >> image;
    	
        sensor_msgs::msg::Image::SharedPtr message =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image)
                .toImageMsg();

	message->header.stamp = this->get_clock()->now();
	message->header.frame_id = "webcam";
	publisher_->publish(*message.get());
    }
    
    Mat image ;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    size_t count_;
    uint32_t sequence;
    VideoCapture cap;
    cv_bridge::CvImagePtr cv_ptr;
};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Cam_pub>());
	rclcpp::shutdown();
	return 0;
}
