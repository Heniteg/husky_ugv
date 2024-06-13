#include <fstream>
#include <iostream>
#include <sys/stat.h>  // For mkdir
#include <errno.h>  // For error code from mkdir
#include <string> 
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <husky_odometry/DistanceAndOrientation.h>
#include <husky_odometry/RelativeCoordinates.h>


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, ros::Publisher& coord_pub, ros::Publisher& dist_orient_pub, std::ofstream& outfile) 
{
    static double start_x = 0.0, start_y = 0.0;
    static bool is_start_set = false;

    if (!is_start_set) 
    {
        start_x = msg->pose.pose.position.x;
        start_y = msg->pose.pose.position.y;
        is_start_set = true;
    }

    double rel_x = msg->pose.pose.position.x - start_x;
    double rel_y = msg->pose.pose.position.y - start_y;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double yaw_deg = yaw * (180.0 / M_PI);

    husky_odometry::RelativeCoordinates coord_msg;
    coord_msg.x = rel_x;
    coord_msg.y = rel_y;
    coord_msg.yaw = yaw_deg;
    coord_pub.publish(coord_msg);

    double distance = sqrt(rel_x*rel_x + rel_y*rel_y);

    husky_odometry::DistanceAndOrientation dist_orient_msg;
    dist_orient_msg.distance = distance;
    dist_orient_msg.orientation_degrees = yaw_deg;
    dist_orient_pub.publish(dist_orient_msg);

    // Write to file
    if (outfile.is_open()) {
        outfile << "Coordinates (x: " << rel_x << ", y: " << rel_y << ", Yaw: " <<yaw_deg<<"), "
                <<"Distance: "<< distance << " meters, Yaw: " << yaw_deg <<" degrees" << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_data_saving");
    ros::NodeHandle nh;

    std::ofstream outfile;
    std::string directory = "/home/heni/ros1_noetic/src/husky_ugv/husky_odometry/saving-robot-information";
    std::string filename = "data.log"; // default file name

    mkdir(directory.c_str(), 0777); // Create a directory with wide-open permissions

    if (errno != 0 && errno != EEXIST)
    {
        ROS_ERROR("Failed to create directory: %s", strerror(errno));
        return 1;
    }

    // Checking command line argument
    for (int i=1; i<argc; ++i)
    {
        std::string argu = argv[i];
        if (argu == "-h" || argu == "--help")
        {
            ROS_INFO("Usage: rosrun husky_odometry_saving_robot_info [options] \n"
                     "Optioins: \n"
                     "  -h, --help              Show this help message and exit \n"
                     "  -f, --file <filename>   Specify the filename to log data");
            return 0;
        }
        else if (argu == "-f" || argu == "-file")
        {
            if (i + 1 < argc)
            {
                filename = argv[++i] + std::string(".log");  // Update filename with input
            }
            else
            {
                ROS_ERROR("--file option requires one argument.");
                return 1;
            }
        }
    }

    std::string filepath = directory + "/" + filename;
    outfile.open(filepath, std::ios::out);

    if (!outfile.is_open())
    {
        ROS_ERROR("Failed to open file: %s for writing.", filepath.c_str());
        return 1;
    }
    else
    {
        std::cout<<"=======================The log file starts to save under the directory below===================="<<std::endl;
        std::cout<<"The <filename>.log file is saved under: /home/heni/ros1_noetic/src/husky_ugv/husky_odometry/saving-robot-information/"<< filename <<std::endl;
    }

    ros::Publisher coord_pub = nh.advertise<husky_odometry::RelativeCoordinates>("relative_coordinates", 10);
    ros::Publisher dist_orient_pub = nh.advertise<husky_odometry::DistanceAndOrientation>("distance_orientation", 10);
    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, boost::bind(odomCallback, _1, coord_pub, dist_orient_pub, boost::ref(outfile)));

    ros::spin();

    outfile.close();

    return 0;
}



