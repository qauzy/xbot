#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"

class Obstacle
{
public:
    // Tunable parameters 可调参数
    const double FORWARD_SPEED = 0.25;
    double angular_velocity;    //随机获得角速度
    const double MIN_SCAN_ANGLE = -15.0/ 360 * M_PI;   //-15度的rad
    const double MAX_SCAN_ANGLE = 15.0 / 360 * M_PI;
    // Should be smaller than sensor_msgs::LaserScan::range_max
    const float MIN_DIST_FROM_OBSTACLE = 0.4;   //离物体的最小距离
    Obstacle();//Stopper();
    void startMoving();

private:
    ros::NodeHandle node;
    ros::Publisher commandPub; // Publisher to the robot~s velocity command topic
    ros::Subscriber laserSub;  // Subscriber to the robot~s laser scan topic
    bool keepMoving;           // Indicates whether the robot should continue moving ，0 or 1
    bool keepMoving0;       //keepMoving的上一个状态
    bool getRandom;             //当keepMoving 由1变为0时，getRandom这一瞬间为真,让我想起数电的激励方程
    //指示机器人应继续移动，0还是1
    void moveForward();
    void moveBackward();
    void turnCorner();//
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan); //Ptr是pointer
};
