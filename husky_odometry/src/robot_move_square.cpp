#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <cmath>


class SquareMove
{
public:
    SquareMove(): 
        loop_rate(10),   // set the loop rate to 10Hz
        counter(0),      // Counter to manage the timing of motions
        move_stage(0),   // Variable to control the stage of the square movement
        rotations(0)    // Count how many times the square has been completed
    {
        // Initialize the publisher and subscriber
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);
        odom_sub = nh.subscribe("/odometry/filtered", 10, &SquareMove::odomCallback, this);

        // Open a file to save odometry data
        outfile.open("robot_odometry_data.log");
    }

    ~SquareMove()
    {
        // Close the file stream if it's open
        if(outfile.is_open())
        {
            outfile.close();
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // Callback to process each incoming odometry message
        if(outfile.is_open())
        {
            // Write position and orientation data to file
            outfile << "Position (" << msg->pose.pose.position.x << ", "<< msg->pose.pose.position.y << "), "
                    << "Orientation (z: " << msg->pose.pose.orientation.z << ")"<< std::endl;
        }
    }

    void moveSquare()
    {
        // Main control loop for moving the robot in a square
        while (ros::ok() && rotations < 3)
        {
            geometry_msgs::Twist cmd;

            // Move forward in 4 straight lines
            if(move_stage<4)
            {
                cmd.linear.x = 0.5;     // Set forward speed
                cmd.angular.z = 0.0;    // No angular velocity
                cmd_vel_pub.publish(cmd);
                // Adjust for timing and distance
                if(++counter >= 50)
                {
                    counter = 0;
                    move_stage++;   // Move to the next stage
                }
            }
            // Turn at 4 courners
            else if(move_stage >= 4 && move_stage < 8)
            {
                cmd.linear.x = 0.0;     // No linear velocity
                cmd.angular.z = 0.75;   // Set turning speed
                cmd_vel_pub.publish(cmd);

                // Adjust for timing and angle
                if(++counter >= 20)
                {
                    counter = 0;
                    move_stage++;
                    if(move_stage >= 8)     // Reset after a complete rotation
                    {
                        move_stage = 0;
                        rotations++;        // Increment the square completion count
                    }
                }
            }

            ros::spinOnce();        // Handle ROS events
            loop_rate.sleep();      // Sleep to maintain loop rate
        }

    }

private:
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;  // Publisher for cmd_vel
    ros::Subscriber odom_sub;    // Subscribe for odom
    ros::Rate loop_rate;        // ROS loop rate
    std::ofstream outfile;      // File stream for saving odometry data
    int counter;                // Timing counter for managing movement stages
    int move_stage;             // Current stage in the movement sequence
    int rotations;              // Number of completed function
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "move_square");   // Initialize ROS with node name
    SquareMove square_move;     // Create an instance of SquareMove
    square_move.moveSquare();   // Start the movement function

    return 0;
}