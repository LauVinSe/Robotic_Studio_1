#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>

class LaserScan : public rclcpp::Node{

    public:
    LaserScan() : Node("laserscan_listener"){
        sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScan::laserscan_callback, this, std::placeholders::_1));
        // Publisher for the modified image
        pub1_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/modified_scan", 10);

        // For lab solution
        pub2_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/modified_scan1", 10);
        pub3_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/modified_scan2", 10);
    }

    private:
    void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const{

        // Solution for LAB Week 2
        // Display the range at a given angle
        int angle_index = 90;
        if (angle_index >= 0 && angle_index < msg->ranges.size()) {
            double angle_at_index = msg->angle_min + angle_index * msg->angle_increment;
            double angle_at_index_degrees = angle_at_index * 180.0 / M_PI; // Convert to degrees
            std::cout << "Range at angle " << angle_at_index << " radians (" 
                      << angle_at_index_degrees << " degrees): " 
                      << msg->ranges.at(angle_index) << " meters" << std::endl;
        }

        // Select a subset of range values within the angular range [-π/4, π/4]
        double angle_min_deg_1 = 0.0;
        double angle_max_deg_1 = 45.0;
        double angle_min_deg_2 = 315.0;
        double angle_max_deg_2 = 360.0;

        // Convert to Radians
        double angle_min_rad_1 = angle_min_deg_1 * M_PI / 180.0;
        double angle_max_rad_1 = angle_max_deg_1 * M_PI / 180.0;
        double angle_min_rad_2 = angle_min_deg_2 * M_PI / 180.0;
        double angle_max_rad_2 = angle_max_deg_2 * M_PI / 180.0;

        // Set the starting and ending index for the subset (create a cone)
        int start_index_1 = static_cast<int>((angle_min_rad_1 - msg->angle_min) / msg->angle_increment);
        int end_index_1 = static_cast<int>((angle_max_rad_1 - msg->angle_min) / msg->angle_increment);
        int start_index_2 = static_cast<int>((angle_min_rad_2 - msg->angle_min) / msg->angle_increment);
        int end_index_2 = static_cast<int>((angle_max_rad_2 - msg->angle_min) / msg->angle_increment);

        // Initialise the subset and populise it
        auto subset_scan_1 = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
        subset_scan_1->ranges = std::vector<float>(msg->ranges.begin() + start_index_1, msg->ranges.begin() + end_index_1 + 1);
        subset_scan_1->angle_min = msg->angle_min + start_index_1 * msg->angle_increment;
        subset_scan_1->angle_max = msg->angle_min + end_index_1 * msg->angle_increment;

        auto subset_scan_2 = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
        subset_scan_2->ranges = std::vector<float>(msg->ranges.begin() + start_index_2, msg->ranges.begin() + end_index_2 + 1);
        subset_scan_2->angle_min = msg->angle_min + start_index_2 * msg->angle_increment;
        subset_scan_2->angle_max = msg->angle_min + end_index_2 * msg->angle_increment;

        // Publis the subset
        pub2_->publish(*subset_scan_1);
        pub3_->publish(*subset_scan_2);

        // // Solution for SPRINT 1
        auto modified_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
        modified_scan->ranges.clear();

        int n = 5; // Set how many points to skip

        // Adjust the angle increment for the new modified scan
        modified_scan->angle_increment = msg->angle_increment * n;

        for (size_t i = 0; i < msg->ranges.size(); i += n){
            modified_scan->ranges.push_back(msg->ranges.at(i));
        }

        // Publish the sparse scan
        pub1_->publish(*modified_scan);

    
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub1_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub2_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub3_;
};

int main (int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScan>());
    rclcpp::shutdown();
    return 0;
}