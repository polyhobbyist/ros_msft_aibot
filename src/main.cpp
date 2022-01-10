#include <math.h>
#include <angles/angles.h>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>

double kAngleScale = 0.5;
double kCameraPixelWidth = 416.0;
double kFOV = 70.0;
double kDegreePerCameraPixel = kFOV / kCameraPixelWidth;
double targetYaw = 0.0;

using std::placeholders::_1;

class FollowMeNode : public rclcpp::Node
{
  public:
    FollowMeNode()
    : Node("ros_msft_aibot")
    {
      markerSub = this->create_subscription<visualization_msgs::msg::Marker>(
        "visual_markers", 10, std::bind(&FollowMeNode::marker_callback, this, _1));

      velocityPub =  this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    }

  private:
    void marker_callback(const visualization_msgs::msg::Marker::SharedPtr msg) const
    {
        targetYaw = 0.0f;

        double degree = (msg->pose.position.x * kDegreePerCameraPixel) - (kFOV / 2.0f);

        targetYaw = angles::from_degrees(-degree);

        RCLCPP_INFO(this->get_logger(), "Turning Towards %f", targetYaw);

        geometry_msgs::msg::Twist vel_msg;
        vel_msg.angular.z = kAngleScale * targetYaw;
        velocityPub->publish(vel_msg);
    }


    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr markerSub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocityPub;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<FollowMeNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;        
}