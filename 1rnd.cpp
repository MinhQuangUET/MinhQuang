#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

#include <iostream>
#include <algorithm>

using namespace std;

const float PI = 3.14159265;
const float tolerance = 1e-2;
float rate = 30;
turtlesim::Pose current_pose;
ros::Publisher pub;

// Tạo hàm tính khoảng cách giữa current_pose và mục tiêu cần đến.
double distance(turtlesim::Pose pose, double goal_x, double goal_y)
{
    double dis = sqrt(pow(goal_x - pose.x, 2) + pow(goal_y - pose.y, 2));
    if(dis < tolerance) dis = 0;
    return dis;
}

// Tạo hàm xác định góc giữa current_pose và mục tiêu cần đến.
double angular(turtlesim::Pose pose, double goal_x, double goal_y)
{
    double goc;
    if (distance(pose, goal_x, goal_y) < tolerance) goc = 0;
    else
    {
        goc = asin((cos(pose.theta)*(goal_y - pose.y) - sin(pose.theta)*(goal_x - pose.x))/
                distance(pose, goal_x, goal_y));
    }
    return goc;
}

// Hàm getMessage khoảng cách và góc 
geometry_msgs::Twist getMessage(double linear_x, double angular_z)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    return msg;
}

void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    current_pose = *msg;
}

int main(int argc, char** argv)
{
    // Khởi tạo argc argv và node 
    ros::init(argc, argv, "myturtle_control");
    ros::NodeHandle h;
    pub = h.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    ros::Subscriber sub = h.subscribe("/turtle1/pose", 1000, poseCallback);
    ros::Rate loopRate(rate);

    double x0 = current_pose.x;
    double y0 = current_pose.y;
    
    int i = 1;

    while (ros::ok())
    {
        if (distance(current_pose, x0, y0) < tolerance)
        {
            // Dừng robot khi đến đủ gần
            pub.publish(getMessage(0,0));
            if(i < argc - 1)
            {
                x0 = atof(argv[i]);
                y0 = atof(argv[i+1]);
                i += 2;
            }
            else break;
        }

        double khoangCach = distance(current_pose, x0, y0);
        double angle = angular(current_pose, x0, y0);

        // Xét xem rùa đi lùi hay đi tiến 
            double dx0 = x0 - current_pose.x;
            double dy0 = y0 - current_pose.y;
            double theta0 = current_pose.theta;
            double dist = sqrt( pow(dx0, 2) + pow(dy0, 2) );
            double angu = acos((cos(theta0)*dx0+sin(theta0)*dy0)/dist);
            double heso;
                if(angu < PI/2) heso = 1;
                else heso = -1;


        loopRate.sleep();
        ros::spinOnce();
        cout << current_pose.x << " " << current_pose.y << " " << current_pose.theta << endl;
        
        geometry_msgs::Twist msg = getMessage(
            min(heso*2.6*khoangCach,  7.0),
            heso*5*angle
        );

        pub.publish(msg);
    }   
    return 0;
}
