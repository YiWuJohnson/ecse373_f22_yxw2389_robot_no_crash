#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <sstream>

ros::Publisher *p_pub;
double wall_dis;

void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    p_pub->publish(msg);
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if(msg->range_min < wall_dis){ROS_INFO("stop going");}
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "robot_publisher");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

  ros::NodeHandle n;
  // Create a float variable to hold the parameter
  // This time initialize it to a default value
  double wall_dist = 1.0;
  // Announce the value of wall_dist before the first call to the Parameter Server
  ROS_INFO_ONCE("wall_dist began with: [%2.2f]", wall_dist);
  // Get the parameter using the node handle that can be updated
  if (n.getParamCached("wall_dist", wall_dist)) {
  ROS_INFO("wall_dist was updated to: [%2.2f]", wall_dist);
}
  // Announce the value of wall_dist after the first call to the Parameter Server
  ROS_INFO_ONCE("wall_dist is now: [%2.2f]", wall_dist);
  wall_dis = wall_dist;
  ros::Subscriber sub = n.subscribe<geometry_msgs::Twist>("des_vel", 50, chatterCallback);
  ros::Subscriber lidarsub = n.subscribe<sensor_msgs::LaserScan>("lidar_1", 50, lidarCallback);
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 50);
  p_pub = &pub;

  ros::Rate loop_rate(10);

  geometry_msgs::Twist msg;

  pub.publish(msg);

  return 0;
}

