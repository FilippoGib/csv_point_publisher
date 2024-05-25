#include "csv_point_publisher/csv_point_publisher_node.h"

CSVPointPublisher::CSVPointPublisher() : Node("csv_point_publisher")
{
    this->declare_parameter<std::string>("blue_csv_file_path", "");
    blue_csv_file_path_ = this->get_parameter("blue_csv_file_path").get_value<std::string>();

    this->declare_parameter<std::string>("red_csv_file_path", "");
    red_csv_file_path_ = this->get_parameter("red_csv_file_path").get_value<std::string>();

    this->declare_parameter<std::string>("publish_topic", "csv/cones_positions");
    publish_topic_ = this->get_parameter("publish_topic").get_value<std::string>();

    this->declare_parameter<int>("publish_frequency", 2);
    int frequency = this->get_parameter("publish_frequency").get_value<int>();
    auto period = std::chrono::milliseconds(1000 / frequency);

    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(publish_topic_, 10);

    // Load and store markers for blue and red cones
    load_points_from_csv(blue_csv_file_path_, markers_, 0.0, 0.0, 1.0); // Blue cones
    load_points_from_csv(red_csv_file_path_, markers_, 1.0, 0.0, 0.0); // Red cones

    timer_ = this->create_wall_timer(period, std::bind(&CSVPointPublisher::publish_markers, this));
}

void CSVPointPublisher::load_points_from_csv(const std::string& file_path, std::vector<visualization_msgs::msg::Marker>& markers, float r, float g, float b)
{
    std::ifstream file(file_path);
    std::string line;
    unsigned int id = markers.size();  // Start IDs from the current size to ensure uniqueness

    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
        return;
    }

    std::getline(file, line);  // Skip header
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        double x, y;
        char delim;

        ss >> x >> delim >> y;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "csv_cones";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0;  // Z coordinate is set to 0
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;  // Size of the marker
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;

        markers.push_back(marker);
    }

    file.close();
}

void CSVPointPublisher::publish_markers()
{
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers = markers_;
    publisher_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "Published %zu markers", markers_.size());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CSVPointPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
