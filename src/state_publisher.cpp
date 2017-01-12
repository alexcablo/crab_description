#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Rate loop_rate(30);

  // Positions:

  double pos_femr1 = 10;

  //message declarations

  sensor_msgs::JointState joint_state;

  while (ros::ok()) {

    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(10);
    joint_state.position.resize(10);
    joint_state.name[0] = "femur_joint_r1";
    joint_state.position[0] = pos_femr1;

    joint_pub.publish(joint_state);

    ros::spin();
    loop_rate.sleep();
}

  return 0;
}

//['coxa_joint_r1','femur_joint_r1','tibia_joint_r1','coxa_joint_r2','femur_joint_r2','tibia_joint_r2','coxa_joint_r3','femur_joint_r3','tibia_joint_r3','coxa_joint_l1','femur_joint_l1','tibia_joint_l1','coxa_joint_l2','femur_joint_l2','tibia_joint_l2','coxa_joint_l3','femur_joint_l3','tibia_joint_l3']
//(10.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)
