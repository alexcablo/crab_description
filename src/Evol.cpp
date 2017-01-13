
#include "ros/ros.h"
#include <stdlib.h>
#include <ctime>
#include "std_msgs/String.h"
#include "Algorithm_lib/NEAT.h"
#include <sstream>
#include "Algorithm_lib/debug.h"
#include <std_msgs/Float64.h>

#include <gazebo_msgs/ModelStates>

const std::string suffixes[6] = {"_r1", "_r2", "_r3", "_l1", "_l2", "_l3"};
const std::string names[3] = {"coxa_joint", "femur_joint", "tibia_joint"};



float pwm_desired[18]={};
float pwm_current[18]={};
float distance_U = 0;
int accel_X = 0;
int accel_Y = 0;
int accel_Z = 0;
int gyro_X = 0;
int gyro_Y = 0;
int gyro_Z = 0;

float vel_X=0, vel_Y=0, vel_Z=0;
float X=0, Y=0, Z=0;
ros::Time t_stamp;

float ts = 0.1;
int time_loop_limit = 1000; //(1000*ts)=10 sec

void sensorCallback(gazebo_msgs::ModelStates msg){
  //Sensor
	distance_U = msg.distance_U;
	X = msg.X;
	Y = msg.Y;
	Z = msg.Z;
	vel_X = msg.vel_X;
	vel_Y = msg.vel_Y;
	vel_Z = msg.vel_Z;
	accel_X = msg.accel_X;
	accel_Y = msg.accel_Y;
	accel_Z = msg.accel_Z;
	gyro_X = msg.gyro_X;
	gyro_Y = msg.gyro_Y;
	gyro_Z = msg.gyro_Z;
	t_stamp = msg.t_stamp;
	for (int i=0;i<=11;i++){
		pwm_current[i] = msg.pwm[i];
	}
	printf("Reading sensor data X: %f",X);
}


int main(int argc, char **argv)
{

	#ifdef DEBUG_H_INCLUDED
	ROS_INFO("Debugging active");
	#endif //DEBUG_H_INCLUDED

	ros::init(argc, argv, "Algorithm_node");

	srand (time(NULL));

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

  ros::Subscriber position_sensor = n.subscribe("/gazebo/model_states",1000, pos_sensor_callback);

	ros::Rate loop_rate(1/ts);

  std_msgs::Float64 global_pos_msgs;

	int time_loop = 0;

	float currentTime = 0;
	float InputVec[12] = {};//Input vector to the network
	float OutputVec[12] = {};//Output vector of the network
	float StepSize = 0.1; //Size of the possition change each second
	float Out_hlim = 1.5; //Up limit possition
	float Out_llim = -1.5; //Down limit positon

	float Fitness = 0;

	//Initialize pub/subs



	//Initalization of the pool
	Pool Spidy_pool(12,12);

	#ifdef DEBUG_H_INCLUDED
	ROS_INFO("Pool created");
	#endif //DEBUG_H_INCLUDED

	Spidy_pool.initializePool();

	#ifdef DEBUG_H_INCLUDED
	ROS_INFO("Pool initialized");
	#endif //DEBUG_H_INCLUDED


	while (ros::ok())
	{

		//Start training

		#ifdef DEBUG_ALGO_H_INCLUDED
		ROS_INFO("Specie: %d. Genome: %d",Spidy_pool.currentSpecies,Spidy_pool.currentGenome);
		#endif //DEBUG_ALGO_H_INCLUDED


		//Generate the network for the current Genome
		Spidy_pool.SpeciesVec[Spidy_pool.currentSpecies].GenomesVec[Spidy_pool.currentGenome].generateNetwork();

		#ifdef DEBUG_H_INCLUDED
		ROS_INFO("Network generated");
		#endif //DEBUG_H_INCLUDED

		//Simulation/Execution loop

		time_loop = 0;

		memset(pwm_current, 0, sizeof(pwm_current));

		while(time_loop<time_loop_limit){

			//TODO: Sensor read implementation

			for(int i=0;i<12;i++)
			{
				InputVec[i]=pwm_current[i];
			}

			currentTime = time_loop*ts;

			Spidy_pool.evaluateCurrent(InputVec,OutputVec);


			//Process outputs
			for(int i=0;i<12;i++)
			{
				OutputVec[i] = OutputVec[i]*StepSize;

				pwm_desired[i] += OutputVec[i]*ts;

				if(pwm_current[i]>Out_hlim)
					pwm_desired[i]=Out_hlim;

				if(pwm_current[i]<Out_llim)
					pwm_desired[i]=Out_llim;
			}

			//TODO: Send pwm(pwm_current)

      i = 0;
      for (int name=0; name<3; name++){
        for(int suf=0; suf<6; suf++){
          global_pos_msgs.data = pwm_current[i];

          joint_pub[i].publish(global_pos_msgs);
          ROS_INFO("%f",pwm_current[i]);
          //position[1]++;
          i++;
        }
      }



			ros::spinOnce();

			loop_rate.sleep();
			time_loop++;
		}

		//Calculate fitness

    //Fitness = sqrt(pow())

		//Evaluation ended

		//Spidy_pool.assignfitness(Fitness);

		//Next genome
		if(Spidy_pool.SpeciesVec[Spidy_pool.currentSpecies].GenomesVec.size()==Spidy_pool.currentGenome+1)
		{
    		if(Spidy_pool.SpeciesVec.size()==Spidy_pool.currentSpecies+1){
				#ifdef DEBUG_ALGO_H_INCLUDED
				ROS_INFO("Generating new generation...");
				#endif //DEBUG_ALGO_H_INCLUDED

				Spidy_pool.newGeneration();

				#ifdef DEBUG_ALGO_H_INCLUDED
				ROS_INFO("New Generation: %d",Spidy_pool.generation);
				#endif //DEBUG_ALGO_H_INCLUDED
			}else{
				Spidy_pool.currentSpecies ++;
				Spidy_pool.currentGenome = 0;
    		}
		}else{
      		Spidy_pool.currentGenome++;
		}

	}

  return 0;
}
