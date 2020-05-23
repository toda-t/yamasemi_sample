#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

nav_msgs::Odometry pos;
sensor_msgs::LaserScan scan;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    pos.header = msg->header;
    pos.child_frame_id = msg->child_frame_id;
    pos.pose = msg->pose;
    pos.twist = msg->twist;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_){
    scan.header = scan_->header;
    scan.angle_min = scan_->angle_min;
    scan.angle_max = scan_->angle_max;
    scan.angle_increment = scan_->angle_increment;
    scan.time_increment = scan_->time_increment;
    scan.scan_time = scan_->scan_time;
    scan.range_min = scan_->range_min;
    scan.range_max = scan_->range_max;
    scan.ranges = scan_->ranges;
    scan.intensities = scan_->intensities;
}

int main(int argc,char **argv){
    ros::init(argc, argv, "sample");    //このノードはsampleという名前であるという宣言
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCallback);  //robotのオドメトリを読み込むsubscriberの宣言。ros::spinOnce()の度にodomCallback()が呼び出される。
    // ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);   //URGの計測データを読み込むsubscriberの宣言。ros::spinOnce()の度scanCallback()が呼び出される。
	ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);  //robotにモータに指令値を送るpublisherの宣言。
	ros::Rate loop_rate(50);
    geometry_msgs::Twist cmd_vel_;

	sleep(1);
    ros::spinOnce();
    const nav_msgs::Odometry init_pos = pos;
    std::vector<double> init_rpy(3);
    tf::Quaternion init_quat(init_pos.pose.pose.orientation.x, init_pos.pose.pose.orientation.y, init_pos.pose.pose.orientation.z, init_pos.pose.pose.orientation.w);
    tf::Matrix3x3(init_quat).getRPY(init_rpy[0], init_rpy[1], init_rpy[2]); //クォータニオンからオイラー角への変換

    std::cout << "straight" << std::endl;
    cmd_vel_.linear.x = 0.6;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.linear.z = 0.0;
    cmd_vel_.angular.x = 0.0;   //roll
    cmd_vel_.angular.y = 0.0;   //pitch
    cmd_vel_.angular.z = 0.0;   //yaw
    while(pos.pose.pose.position.x - init_pos.pose.pose.position.x <= 1.0 - 0.005){
        cmd_vel_pub.publish(cmd_vel_);  //cmd_velのpublish
		ros::spinOnce();
		loop_rate.sleep();  
    }

    std::cout << "spin" << std::endl;
    std::vector<double> rpy(3);
    tf::Quaternion quat(pos.pose.pose.orientation.x, pos.pose.pose.orientation.y, pos.pose.pose.orientation.z, pos.pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(rpy[0], rpy[1], rpy[2]);
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.angular.z = 1.0;
    while(rpy[2]-init_rpy[2] >= -M_PI + M_PI/18.0 && rpy[2]-init_rpy[2] <= M_PI - M_PI/18.0){
        cmd_vel_pub.publish(cmd_vel_);
		ros::spinOnce();
        tf::Quaternion quat(pos.pose.pose.orientation.x,pos.pose.pose.orientation.y,pos.pose.pose.orientation.z,pos.pose.pose.orientation.w);
        tf::Matrix3x3(quat).getRPY(rpy[0], rpy[1], rpy[2]);
		loop_rate.sleep();  
    }

    std::cout << "straight" << std::endl;
    cmd_vel_.linear.x = 0.6;
    cmd_vel_.angular.z = 0.0;
    while(pos.pose.pose.position.x - init_pos.pose.pose.position.x >= 0.005){
        cmd_vel_pub.publish(cmd_vel_);
		ros::spinOnce();
		loop_rate.sleep();  
    }

    std::cout << "goal" << std::endl;
    cmd_vel_.angular.z = 0.0;
    cmd_vel_pub.publish(cmd_vel_);

}