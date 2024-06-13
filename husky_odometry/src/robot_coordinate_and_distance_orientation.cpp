#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <husky_odometry/DistanceAndOrientation.h>
#include <husky_odometry/RelativeCoordinates.h>

// Starting pose for reference
double start_x = 0.0, start_y = 0.0;
bool is_start_set = false; // is a flag to ensure that the starting position is recorded only once

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, ros::Publisher& coord_pub, ros::Publisher& dist_orient_pub)
{
    if (!is_start_set)
    {
        start_x = msg->pose.pose.position.x;
        start_y = msg->pose.pose.position.y;
        is_start_set = true;
    }

    // Calculate relative position
    double rel_x = msg->pose.pose.position.x - start_x;
    double rel_y = msg->pose.pose.position.y - start_y;

    // Calculate orientation
    tf::Quaternion q(msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);
    tf::Matrix3x3 eul(q);
    double roll, pitch, yaw;
    eul.getRPY(roll, pitch, yaw);
    double yaw_deg = yaw * (180.0 / M_PI);

    // Publish relative coordinates
    husky_odometry::RelativeCoordinates coord_msg;
    coord_msg.x = rel_x;
    coord_msg.y = rel_y;
    coord_msg.yaw = yaw_deg;
    coord_pub.publish(coord_msg);

    // Calculate distance and orientation
    double distance = sqrt(rel_x*rel_x + rel_y*rel_y);
    husky_odometry::DistanceAndOrientation dist_orient_msg;
    dist_orient_msg.distance = distance;
    dist_orient_msg.orientation_degrees = yaw_deg;
    dist_orient_pub.publish(dist_orient_msg);

    // Display the results
    ROS_INFO("Coordinates:(x: %f, y: %f, yaw: %f), Distance: %f, Orientation: %f (degrees)", rel_x, rel_y, yaw_deg, distance, yaw_deg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_processor");
    ros::NodeHandle nh;
    
    ros::Publisher coord_pub = nh.advertise<husky_odometry::RelativeCoordinates>("relative_coordinates", 1000);
    ros::Publisher dist_orient_pub = nh.advertise<husky_odometry::DistanceAndOrientation>("distance_orientation", 1000);
    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1000, boost::bind(odomCallback, _1, coord_pub, dist_orient_pub));

    ros::spin();
    return 0;
}