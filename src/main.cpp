#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
float kAngleScale = 0.5f;
float kCameraPixelWidth = 416.0f;
float kFOV = 70.0f;
float kDegreePerCameraPixel = kFOV / kCameraPixelWidth;

float targetYaw = 0.0f;

ros::Subscriber markerSub;
ros::Publisher velocityPub;

void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    targetYaw = 0.0f;

	for (auto &marker : msg->markers)
	{
        float degree = (marker.pose.position.x * kDegreePerCameraPixel) - (kFOV / 2.0f);

        targetYaw = (float)angles::from_degrees(-degree);
	}

    ROS_INFO("Turning Towards %f", targetYaw);

	geometry_msgs::Twist vel_msg;
	vel_msg.angular.z = kAngleScale * targetYaw;
	velocityPub.publish(vel_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ai_bot");
	ros::NodeHandle n;

	markerSub = n.subscribe("tracked_objects", 10, markerCallback);
	velocityPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	ros::spin();

	return 0;        
}