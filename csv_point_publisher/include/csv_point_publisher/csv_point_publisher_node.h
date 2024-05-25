#ifndef CSV_POINT_PUBLISHER_H
#define CSV_POINT_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class CSVPointPublisher : public rclcpp::Node
{
public:
    CSVPointPublisher();

private:
    void load_points_from_csv(const std::string& file_path, std::vector<visualization_msgs::msg::Marker>& markers, float r, float g, float b);
    void publish_markers();
    std::vector<visualization_msgs::msg::Marker> markers_;
    std::string blue_csv_file_path_;
    std::string red_csv_file_path_;
    std::string publish_topic_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // CSV_POINT_PUBLISHER_H


