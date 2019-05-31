#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>

#include <iostream>
#include <algorithm>

using namespace std;

const float PI = 3.14159265;
const float tolerance = 1e-2;
float rate = 30;

class Turtles {
public:
    int turtle_idx;
    ros::Subscriber sub;
    ros::Publisher pub;
    turtlesim::Pose current_pose;

    void callback(const turtlesim::Pose::ConstPtr& msg)
    {
        // cout << "turtle " << turtle_idx+1 << " " << msg->x << " " << msg->y << endl;
        current_pose = *msg;
    }
};

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

geometry_msgs::Twist getMessage(double linear_x, double angular_z)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    return msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "myturtle_control"); 
    ros::NodeHandle node; 

    int solu = atoi(argv[1]);
    Turtles turtles[solu];

    for(int i = 0; i < solu; i++)
    {
        if(i != 0)
        {
            ros::service::waitForService("spawn"); //doi mot dich vu ten la "spawn"
            ros::ServiceClient spawner = node.serviceClient<turtlesim::Spawn>("spawn"); //goi dich vu khi nhan duoc "spawn"
            turtlesim::Spawn rua;
            rua.request.x = 5.544445;
            rua.request.y = 5.544445;
            spawner.call(rua);
        }

        stringstream s;
        s << "turtle"  << i + 1;
        string name = s.str();

        turtles[i].pub = node.advertise<geometry_msgs::Twist>(name + "/cmd_vel", 1000);
        turtles[i].turtle_idx = i;
        turtles[i].sub = node.subscribe(name+"/pose", 1000, &Turtles::callback, &turtles[i]);
    }

    double goal[solu][2];
    for(int i = 0; i < solu; i++)
    {
        goal[i][0] = turtles[i].current_pose.x;
        goal[i][1] = turtles[i].current_pose.y;
    }
    int bien = 2;

    ros::Rate loopRate(rate);
    while(ros::ok())
    {
        for(int i = 0; i < solu; i++)
        {
            if(distance(turtles[i].current_pose, goal[i][0], goal[i][1]) < tolerance)
            {
                turtles[i].pub.publish(getMessage(0, 0));
                if(bien < argc - 1)
                {
                    goal[i][0] = atof(argv[bien]);
                    goal[i][1] = atof(argv[bien + 1]);
                    bien += 2;
                    cout << "turtle" << i + 1 << ": " << goal[i][0] << " " << goal[i][1] << endl;
                }
                else break;
            }

            // Xét xem rùa đi lùi hay đi tiến
                double dx0 = goal[i][0] - turtles[i].current_pose.x;
                double dy0 = goal[i][1] - turtles[i].current_pose.y;
                double theta0 = turtles[i].current_pose.theta;
                double dist = sqrt( pow(dx0, 2) + pow(dy0, 2) );
                double angu = acos((cos(theta0)*dx0+sin(theta0)*dy0)/dist);
                double heso;
                    if(angu < PI/2) heso = 1;
                    else heso = -1;


            geometry_msgs::Twist msg = getMessage(
                min(heso*2.6*distance(turtles[i].current_pose, goal[i][0], goal[i][1]),  7.0),
                heso*5*angular(turtles[i].current_pose, goal[i][0], goal[i][1])
            );
            turtles[i].pub.publish(msg);
        }
        
        loopRate.sleep();
        ros::spinOnce();
    }
    return 0;

}
