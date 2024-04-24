#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

/**
 * This code is used to calculate the distance and orientation that the husky robot tranveled from its
 * starting point while we start the program. It will give us the euclidean ditance and the yaw angle from its starting position
*/

// Callback function to process odometry data
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Get the position
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;  // If needed

    // Extracting the quaternion components
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    // Convert quaternion to roll, pitch, yaw if needed
    tf::Quaternion q(qx, qy, qz, qw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Convert yaw from radians to degrees
    double yaw_deg = yaw * (180.0 / M_PI);

    // Calculate distance from the origin
    double distance = sqrt(x*x + y*y);

    // Display the results
    ROS_INFO("Distance: %f, Orientation: %f (degrees)", distance, yaw_deg);
}

int main(int argc, char **argv)
{
    std::cout<<"=======Odometry listener======"<<std::endl;
    ros::init(argc, argv, "odom_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/odometry/filtered", 1000, odomCallback);
    ros::spin();
    return 0;
}