#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>

const std::string suffixes[6] = {"_r1", "_r2", "_r3", "_l1", "_l2", "_l3"};
const std::string names[3] = {"coxa_joint", "femur_joint", "tibia_joint"};
//
// void chatterLegsState (const LegsStateConstPtr& state){
//
// }

int main(int argc, char** argv){
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle n;
  ros::Publisher joint_pub[18];

  std::string joint_name;
  int i =0;
	for (int name=0; name<3; name++){
		for(int suf=0; suf<6; suf++){
			joint_name = "/crab/" + names[name] + suffixes[suf]+ "_position_controller/command";
      joint_pub[i] = n.advertise<std_msgs::Float64>(joint_name, 1000);
      //ROS_INFO("%s",joint_name);
      i++;
    }
  }

  ros::Rate loop_rate(1);

    // Positions:

  float position[18] = {};


  //message declarations

  std_msgs::Float64 global_pos_msgs;

  while (ros::ok()) {

    i = 0;
    for (int name=0; name<3; name++){
      for(int suf=0; suf<6; suf++){
        global_pos_msgs.data = position[i];

        joint_pub[i].publish(global_pos_msgs);
        ROS_INFO("%f",position[i]);
        //position[1]++;
        i++;
      }
    }

    ROS_INFO("Ended time step");

    ros::spinOnce();
    loop_rate.sleep();
}

  return 0;
}

//['coxa_joint_r1','femur_joint_r1','tibia_joint_r1','coxa_joint_r2','femur_joint_r2','tibia_joint_r2','coxa_joint_r3','femur_joint_r3','tibia_joint_r3','coxa_joint_l1','femur_joint_l1','tibia_joint_l1','coxa_joint_l2','femur_joint_l2','tibia_joint_l2','coxa_joint_l3','femur_joint_l3','tibia_joint_l3']
//(10.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)
