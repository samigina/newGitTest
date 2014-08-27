#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "move_to_target_coordinate");
	ros::NodeHandle n;
	ros::Rate r(10.0);

	tf::TransformListener listener;
	ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

	while(ros::ok())
	{
		tf::StampedTransform transform;
		try{
		  listener.waitForTransform("/target_frame", "/base_link", ros::Time(0), ros::Duration(10.0));
		  listener.lookupTransform("/target_frame", "/base_link", ros::Time(0), transform);
		}
		catch(tf::TransformException ex){
		  ROS_ERROR("%s", ex.what());
		 // ros::Duration(1.0).sleep();
		}	
	
	
	geometry_msgs::Twist vel_msg;
	vel_msg.angular.z = 1.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
	vel_msg.linear.x = 0.125 * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));

	vel_pub.publish(vel_msg);
	r.sleep();
	}
	return 0;
}
