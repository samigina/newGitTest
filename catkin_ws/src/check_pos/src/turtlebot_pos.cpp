#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "position_tracker");
  ros::NodeHandle n;
  ros::Rate r(10.0);

  tf::TransformListener listener;

  while(ros::ok())
  {
    tf::StampedTransform xform;
    try
    {
      listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(1.0));
      listener.lookupTransform("/odom", "/base_link", ros::Time(0), xform);
    }
    catch(tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
     // ros::Duration(1.0).sleep();
    }
 

    ROS_INFO("x: %f, y: %f, z: %f", xform.getOrigin().x(), xform.getOrigin().y(), xform.getOrigin().z());
    r.sleep();
  }

  return 0;
}
