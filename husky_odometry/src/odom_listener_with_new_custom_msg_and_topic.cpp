#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <husky_odometry/DistanceAndOrientation.h> // Include your custom message here

/**
 * This code is used to calculate the distance and orientation that the husky robot tranveled from its
 * starting point while we start the program. It will give us the euclidean ditance and the yaw angle from its starting position
 * In addition it contains a custom message to generate a topic which contains the distance and orientation of the robot whenever it is called
*/

double current_distance = 0.0;
double current_yaw_deg = 0.0;

// Callback function to process odometry data
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, ros::Publisher& pub)
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
    current_yaw_deg = yaw * (180.0 / M_PI);

    // Calculate distance from the origin
    current_distance = sqrt(x*x + y*y);

    husky_odometry::DistanceAndOrientation msg_out;
    msg_out.distance = current_distance;
    msg_out.orientation_degrees = current_yaw_deg;

    pub.publish(msg_out);

    // Display the results
    ROS_INFO("Distance: %f, Orientation: %f (degrees)", current_distance, current_yaw_deg);
}

int main(int argc, char **argv)
{
    std::cout<<"=======Odometry listener with custom message and a new topic======"<<std::endl;
    ros::init(argc, argv, "odom_listener_with_custom_msg");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<husky_odometry::DistanceAndOrientation>("distance_orientation", 1000);
    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1000, boost::bind(odomCallback, _1, pub));
    ros::spin();
    return 0;
}