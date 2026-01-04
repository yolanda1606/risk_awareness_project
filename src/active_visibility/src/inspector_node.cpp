#include <rclcpp/rclcpp.hpp>
#include <voxblox_ros/esdf_server.h>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float32.hpp>
#include <voxblox_ros/conversions.h> // Required for deserialization
#include <glog/logging.h>

class UncertaintyInspector : public rclcpp::Node {
public:
  UncertaintyInspector() : Node("uncertainty_inspector") {
    // 1. Subscribe to the raw Voxblox Layer
    tsdf_sub_ = this->create_subscription<voxblox_msgs::msg::Layer>(
        "/tsdf_map_out", 1, std::bind(&UncertaintyInspector::tsdfCallback, this, std::placeholders::_1));

    // 2. Publisher for the "Purple Cloud" (Visual Debugging)
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/uncertainty_cloud", 1);
    
    // 3. Publisher for the Score (For MPC later)
    score_pub_ = this->create_publisher<std_msgs::msg::Float32>("/visibility_score", 1);

    // TARGET: The Hidden Red Box Position (from your SDF file)
    roi_center_ = voxblox::Point(1.15, 0.0, 0.75); 
    roi_size_ = 0.3; // Check a 30cm box around it
  }

  void tsdfCallback(const voxblox_msgs::msg::Layer::SharedPtr layer_msg) {
    voxblox::Layer<voxblox::TsdfVoxel> layer(layer_msg->voxel_size, layer_msg->voxels_per_side);
    
    // FIX 1: Pass 'layer_msg' directly (it is already a pointer), do not dereference with *
    voxblox::deserializeMsgToLayer<voxblox::TsdfVoxel>(layer_msg, &layer);

    int unknown_count = 0;
    visualization_msgs::msg::Marker points_marker;
    points_marker.header.frame_id = "world";
    points_marker.header.stamp = this->now();
    points_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    points_marker.action = visualization_msgs::msg::Marker::ADD;
    points_marker.scale.x = layer.voxel_size();
    points_marker.scale.y = layer.voxel_size();
    points_marker.scale.z = layer.voxel_size();
    points_marker.color.r = 1.0f; points_marker.color.g = 0.0f; points_marker.color.b = 1.0f; // PURPLE
    points_marker.color.a = 0.8f; 

    // Iterate through the Region of Interest (ROI)
    float step = layer.voxel_size();
    for (float x = -roi_size_/2; x <= roi_size_/2; x += step) {
      for (float y = -roi_size_/2; y <= roi_size_/2; y += step) {
        for (float z = -roi_size_/2; z <= roi_size_/2; z += step) {
          voxblox::Point pt = roi_center_ + voxblox::Point(x, y, z);
          
          // FIX 2: Use simpler API 'getVoxelPtrByCoordinates'
          // This avoids the Eigen math errors and Index locking issues
          voxblox::TsdfVoxel* voxel = layer.getVoxelPtrByCoordinates(pt);

          if (voxel) {
              // Voxel exists. Check if it has low weight (Unknown)
              // We use a small threshold. If weight < 1e-2, we haven't seen it enough.
              if (voxel->weight < 1e-2) {
                  unknown_count++;
                  geometry_msgs::msg::Point p_msg;
                  p_msg.x = pt.x(); p_msg.y = pt.y(); p_msg.z = pt.z();
                  points_marker.points.push_back(p_msg);
              }
          } else {
              // Voxel does not exist (Unallocated block) -> Definitely Unknown
              unknown_count++;
              geometry_msgs::msg::Point p_msg;
              p_msg.x = pt.x(); p_msg.y = pt.y(); p_msg.z = pt.z();
              points_marker.points.push_back(p_msg);
          }
        }
      }
    }

    // Publish results
    marker_pub_->publish(points_marker);
    
    std_msgs::msg::Float32 score_msg;
    score_msg.data = (float)unknown_count;
    score_pub_->publish(score_msg);
    
    // Optional: Only print if count changes to reduce spam, or use throttle
     RCLCPP_INFO(this->get_logger(), "Unknown Voxels: %d", unknown_count);
  }

private:
  rclcpp::Subscription<voxblox_msgs::msg::Layer>::SharedPtr tsdf_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr score_pub_;
  voxblox::Point roi_center_;
  double roi_size_;
};

int main(int argc, char * argv[]) {
  // 1. Initialize ROS
  rclcpp::init(argc, argv);

  // 2. Initialize Google Logging (CRITICAL for Voxblox)
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler(); // This prints a stack trace if it crashes
  FLAGS_alsologtostderr = 1; // Print logs to terminal, not just files

  // 3. Print status to prove we are alive
  std::cout << "\n============================================" << std::endl;
  std::cout << "   INSPECTOR NODE STARTED SUCCESSFULLY" << std::endl;
  std::cout << "   Watching Topic: /tsdf_map_out" << std::endl;
  std::cout << "   Publishing To:  /uncertainty_cloud" << std::endl;
  std::cout << "============================================\n" << std::endl;

  // 4. Run
  rclcpp::spin(std::make_shared<UncertaintyInspector>());
  
  rclcpp::shutdown();
  return 0;
}
}