#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<std_msgs::Float64>("/crab/femur_joint_r1_position_controller/command", 1000);
  ros::Rate loop_rate(1);

  // Positions:

  float pos = 1;

  //message declarations

  std_msgs::Float64 femur_joint_r1;

  while (ros::ok()) {

    femur_joint_r1.data = pos;

    joint_pub.publish(femur_joint_r1);
    ROS_INFO("%f",pos);

    pos_femr1++;

    ros::spinOnce();
    loop_rate.sleep();
}

  return 0;
}

//['coxa_joint_r1','femur_joint_r1','tibia_joint_r1','coxa_joint_r2','femur_joint_r2','tibia_joint_r2','coxa_joint_r3','femur_joint_r3','tibia_joint_r3','coxa_joint_l1','femur_joint_l1','tibia_joint_l1','coxa_joint_l2','femur_joint_l2','tibia_joint_l2','coxa_joint_l3','femur_joint_l3','tibia_joint_l3']
//(10.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)
