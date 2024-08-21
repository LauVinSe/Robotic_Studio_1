#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class ImageSubscriber : public rclcpp::Node{

    public:
    ImageSubscriber() : Node("simple_image_subscriber"){
        sub1_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&ImageSubscriber::image_callback, this, 
            std::placeholders::_1));
        // Publisher for the modified image
        pub1_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_modified", 10);
    }

    private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const{
        try{
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            // Draw a circle in the middle of the image
            cv::Point center(frame.cols / 2, frame.rows / 2);  // Center point of the image
            int radius = 50;  // Radius of the circle
            cv::Scalar color(0, 255, 0);  // Green color in BGR
            int thickness = 2;  // Thickness of the circle's outline

            cv::circle(frame, center, radius, color, thickness);

            cv::imshow("Turtlebot Camera", frame);
            cv::waitKey(10);

            // Convert the modified OpenCV Mat back to a ROS image message
            sensor_msgs::msg::Image::SharedPtr out_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

            // Publish the modified image
            pub1_->publish(*out_msg.get());
        }
        catch (cv_bridge::Exception& e){
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub1_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub1_;
};

int main (int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}