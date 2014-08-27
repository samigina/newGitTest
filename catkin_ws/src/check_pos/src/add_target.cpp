#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "add_new_frame");
	ros::NodeHandle node;

	tf::TransformBroadcaster br;
	tf::Transform transform;

	ros::Rate r(10.0);
	while(node.ok())
	{
		transform.setOrigin( tf::Vector3(2.0, 2.0, 0.0102));
		transform.setRotation( tf::Quaternion(0, 0, 0) );
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "/target_frame"));
	r.sleep();
	}
	return 0;
}
