#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <sstream>

ros::Publisher *p_pub;
int stop_moving = 0;

void desvelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  geometry_msgs::Twist to send = *msg;
  if(stop_moving & (to_send.linear.x > 0.0 )){
    to_send.linear.x =0.0 ;
    ROS_INFO_THROTTLE(0.5, "Stop moving !!!")
  }
  p_pub -> publish(to_send);
 }
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  stop_moving=0;
   for(int indx = 45 ; indx < 225; indx ++)
   {
     if(msg->ranges[indx] < 0.75){
        stop_moving =1; break;
       }
   };
   if (stop_moving){
     ROS_INFO_THROTTLE(0.5, "Stop moving !!!")
   };
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
  ros::Subscriber sub = n.subscribe("robot0/laser_0", 10, LaserScanCallback);
  ros::Subscriber sub1 = n.subscribe("robot0/des_vel", 10, desvelCallback);
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1000);

  p_pub = &cmd_vel_pub ;

  ros::Rate loop_rate(10);

  pub.publish(msg);

  return 0;
}

