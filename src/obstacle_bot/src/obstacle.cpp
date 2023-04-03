#include "obstacle.h"
#include "geometry_msgs/Twist.h"

#include<stdlib.h>  //随机数
#include<time.h>
#define random(x) (rand()%x)

/*void main()
{

     srand((int)time(0));
     for(int x=0;x<10;x++)
           printf("%d/n",random(100));
}*/

Obstacle::Obstacle()
{
    keepMoving = true;
    keepMoving0=true;
    getRandom = false;
    // Advertise a new publisher for the robot~s velocity command topic
    commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // Subscribe to the simulated robot~s laser scan topic
    laserSub = node.subscribe("/scan", 1, &Obstacle::scanCallback, this);

    ros::Subscriber subimu = node.subscribe("/imu/data", 1000, &Obstacle::imuCallback,this);
}
// Send a velocity command
void Obstacle::moveForward()
{
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = FORWARD_SPEED;
    ROS_INFO("SPEED : % f", msg.linear.x);
    commandPub.publish(msg);
}

void Obstacle::turnCorner()
{
    static geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    if(keepMoving0==1 && keepMoving==0)
    getRandom = 1;
    else
    getRandom = 0;

    if(getRandom)   //如果正在转弯就不用重新生成随机数，只有转弯的第一个时刻产生随机数
    {
        angular_velocity = (random(100)-50)/25.0;
        msg.angular.z = angular_velocity;
    }
    ROS_INFO("Angular velocity : % f", msg.angular.z);
    commandPub.publish(msg);
}

// Process the incoming laser scan message
void Obstacle::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
/*
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min   //start angle of the scan [rad]
float32 angle_max
float32 angle_increment //angular distance between measurements [rad]
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
*/
{
    keepMoving0 = keepMoving;
    bool isObstacleInFront = false; //是前面的障碍物吗
    keepMoving = true;
    // Find the closest range between the defined minimum and maximum angles

    /**************************************************************************************************/
    int minIndex = floor((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    //floor() 向下取整
    //将弧度制变为360度制，这里在 MIN_SCAN_ANGLE = -15.0 / 180 * M_PI时minndex为345度
    //功能：返回一个小于传入参数的最大整数
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    //功能：scan->angle_min表示实际扫描时开始的rad，
    //为什么要-scan->angle_min :因为这样就与坐标系无关，参照系就变为了以扫描的开始的那条线为基准，像右边那样   \|/
    //化为度数
    /**********************************************************************************************/
   //  ROS_INFO("Turn a corner!");
    std::vector<std::pair<float, float>> scaled_ranges;
    //pair是将2个数据组合成一个数据，
    for (int i = 0; i < floor(2*M_PI/scan->angle_increment); i++)
    {
        if(std::isinf(scan->ranges[i]) == false){

            ROS_INFO("Turn a corner! minIndex->%d, maxIndex->%d,scan->angle_increment->%f,scan->angle_min->%f, %d:%f",minIndex,maxIndex,scan->angle_increment,scan->angle_min,i,scan->ranges[i]);

        }

    }

    for (int i = minIndex; i < maxIndex; i++)
        scaled_ranges.push_back(std::make_pair(i, scan->ranges[i]));//make_pair整合到一个容器
//    for (int i = 0; i <= maxIndex; i++)
//        scaled_ranges.push_back(std::make_pair(i, scan->ranges[i]));
    int cnt = 0;
    for (auto value : scaled_ranges)
    {
        // std::cout << value.first << ":" << value.second << ",";//应该是value的第一个值与第二个值
         //第一个值表示角度345-15，第二个表示距离
        if (value.second < MIN_DIST_FROM_OBSTACLE)
        {   //cnt++;
            keepMoving = false;
            ROS_INFO("Turn a corner! %d:",sizeof(scan->ranges)/sizeof(scan->ranges[0]));
            break;

        }
    }
    
     //std::cout << std::endl;
}

void Obstacle::moveBackward()
{
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = -FORWARD_SPEED;
    ROS_INFO("SPEED : % f", msg.linear.x);
    commandPub.publish(msg);
}


void Obstacle::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    ROS_INFO("imu: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
             msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
             msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
}


void Obstacle::startMoving()
{
    srand((int)time(0));

    ros::Rate rate(10);
    ROS_INFO("Start moving");
    // Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
    while (ros::ok())
    {
        // Need to call this function often to allow ROS to process incoming messages
        ros::spinOnce();
        
        if (false == keepMoving)
            moveBackward();
        else
            moveForward();
        
        rate.sleep();
    }
}


