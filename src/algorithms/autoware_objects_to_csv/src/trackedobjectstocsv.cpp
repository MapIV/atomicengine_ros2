#include "rclcpp/rclcpp.hpp"
#include "autoware_perception_msgs/msg/tracked_objects.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>

using namespace std;
using namespace std::chrono_literals;
using TrackedObjects = autoware_perception_msgs::msg::TrackedObjects;
using TwistMsg = geometry_msgs::msg::TwistWithCovarianceStamped;

class TrackedObjectsToCSV : public rclcpp::Node {
public:
    TrackedObjectsToCSV() : Node("tracked_objects_to_csv"), sequence_number(-1), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        this->declare_parameter<std::string>("csv_directory_path", "/default/path/to/directory");
        this->declare_parameter<std::string>("input_topic", "/tracked_objects");
        this->declare_parameter<std::string>("twist_topic", "/twist_topic");
        this->declare_parameter<std::string>("lidar_frame", "lidar_frame");

        std::string directory_path;
        this->get_parameter("csv_directory_path", directory_path);
        std::string input_topic;
        this->get_parameter("input_topic", input_topic);
        std::string twist_topic;
        this->get_parameter("twist_topic", twist_topic);
        this->get_parameter("lidar_frame", lidar_frame_);

        // Generate the timestamped filenames
        objects_csv_file_path_ = directory_path + "/" + generate_timestamped_filename("tracked_objects");
        vehicle_csv_file_path_ = directory_path + "/" + generate_timestamped_filename("vehicle_pose");

        RCLCPP_INFO_STREAM(this->get_logger(), "Launching TrackedObjectsToCSV node with tracked objects CSV: " << objects_csv_file_path_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Vehicle pose will be logged in: " << vehicle_csv_file_path_);

        rclcpp::QoS qos(rclcpp::KeepLast(100));
        qos.best_effort();

        tracked_objects_subscription_ = this->create_subscription<TrackedObjects>(
            input_topic,
            qos,
            bind(&TrackedObjectsToCSV::listener_callback, this, placeholders::_1)
        );

        twist_subscription_ = this->create_subscription<TwistMsg>(
            twist_topic,
            qos,
            bind(&TrackedObjectsToCSV::twist_callback, this, placeholders::_1)
        );

        initialize_csvs();
    }

private:
    void initialize_csvs() {
        // Initialize tracked objects CSV
        ofstream csvfile(objects_csv_file_path_, ios::out | ios::trunc);
        csvfile << "timestamp,sequence_number,lidar_frame_id,lidar_x,lidar_y,lidar_z,"
                << "object_frame_id,object_id,object_class,x_position,y_position,z_position,"
                << "x_dimension,y_dimension,z_dimension,quaternion_x,quaternion_y,quaternion_z,quaternion_w,"
                << "velocity_x,velocity_y,velocity_z," << endl;
        csvfile.close();

        // Initialize vehicle pose CSV
        ofstream vehicle_csvfile(vehicle_csv_file_path_, ios::out | ios::trunc);
        vehicle_csvfile << "timestamp,frame_id,sequence_number,x_position,y_position,z_position,quaternion_x,quaternion_y,quaternion_z,quaternion_w,"
                        << "velocity_x,velocity_y,velocity_z,"
                        << "angular_velocity_x,angular_velocity_y,angular_velocity_z" << endl;
        vehicle_csvfile.close();
    }

    void twist_callback(const TwistMsg::SharedPtr msg) {
        latest_twist_ = *msg;  // Save the latest twist message
    }

    void listener_callback(const TrackedObjects::SharedPtr msg) {
        // Write tracked objects data
        ofstream csvfile(objects_csv_file_path_, ios::out | ios::app);
        sequence_number++;

        double timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) / 1e9;

        // Get the lidar frame to map transform
        geometry_msgs::msg::TransformStamped lidar_transform;
        try {
            lidar_transform = tf_buffer_.lookupTransform("map", lidar_frame_, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }

        for (auto &obj : msg->objects) {
            csvfile << fixed << setprecision(9) << timestamp << ","
                    << sequence_number << ","
                    << lidar_frame_ << ","
                    << lidar_transform.transform.translation.x << ","
                    << lidar_transform.transform.translation.y << ","
                    << lidar_transform.transform.translation.z << ","
                    << msg->header.frame_id << ","
                    << to_hex_string(obj.object_id.uuid) << ","
                    << (obj.classification.empty() ? "Unknown" : to_string(obj.classification[0].label)) << ","
                    << obj.kinematics.pose_with_covariance.pose.position.x << ","
                    << obj.kinematics.pose_with_covariance.pose.position.y << ","
                    << obj.kinematics.pose_with_covariance.pose.position.z << ","
                    << obj.shape.dimensions.x << ","
                    << obj.shape.dimensions.y << ","
                    << obj.shape.dimensions.z << ","
                    << obj.kinematics.pose_with_covariance.pose.orientation.x << ","
                    << obj.kinematics.pose_with_covariance.pose.orientation.y << ","
                    << obj.kinematics.pose_with_covariance.pose.orientation.z << ","
                    << obj.kinematics.pose_with_covariance.pose.orientation.w << ","
                    << obj.kinematics.twist_with_covariance.twist.linear.x << ","
                    << obj.kinematics.twist_with_covariance.twist.linear.y << ","
                    << obj.kinematics.twist_with_covariance.twist.linear.z << endl;
        }

        ofstream vehicle_csvfile(vehicle_csv_file_path_, ios::out | ios::app);

        geometry_msgs::msg::TransformStamped vehicle_transform;
        try {
            vehicle_transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            vehicle_csvfile << fixed << setprecision(9) << timestamp << ","
                            << msg->header.frame_id << ","
                            << sequence_number << ","
                            << vehicle_transform.transform.translation.x << ","
                            << vehicle_transform.transform.translation.y << ","
                            << vehicle_transform.transform.translation.z << ","
                            << vehicle_transform.transform.rotation.x << ","
                            << vehicle_transform.transform.rotation.y << ","
                            << vehicle_transform.transform.rotation.z << ","
                            << vehicle_transform.transform.rotation.w << ","
                            << latest_twist_.twist.twist.linear.x << ","
                            << latest_twist_.twist.twist.linear.y << ","
                            << latest_twist_.twist.twist.linear.z << ","
                            << latest_twist_.twist.twist.angular.x << ","
                            << latest_twist_.twist.twist.angular.y << ","
                            << latest_twist_.twist.twist.angular.z << endl;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    string to_hex_string(const std::array<uint8_t, 16>& uuid) {
        stringstream ss;
        for (auto byte : uuid) {
            ss << hex << setw(2) << setfill('0') << (int)byte;
        }
        return ss.str();
    }

    std::string generate_timestamped_filename(const std::string& prefix) {
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << prefix << "_" << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << ".csv";
        return ss.str();
    }

    rclcpp::Subscription<TrackedObjects>::SharedPtr tracked_objects_subscription_;
    rclcpp::Subscription<TwistMsg>::SharedPtr twist_subscription_;
    
    int sequence_number;
    std::string objects_csv_file_path_;
    std::string vehicle_csv_file_path_;
    
    geometry_msgs::msg::TwistWithCovarianceStamped latest_twist_;
    std::string lidar_frame_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = make_shared<TrackedObjectsToCSV>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
