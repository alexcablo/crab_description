
#include "ros/ros.h"
#include "ros/time.h"
#include <stdlib.h>
#include <ctime>
#include "std_msgs/String.h"
#include "Algorithm_lib/NEAT.h"
#include <sstream>
//#include "Algorithm_lib/debug.h"
#include "Algorithm_lib/debug_algorithm.h"
#include <std_msgs/Float64.h>
#include "std_srvs/Empty.h"
#include <sstream>

#include <gazebo_msgs/ModelStates.h>

const std::string suffixes[6] = {"_r1", "_r2", "_r3", "_l1", "_l2", "_l3"};
const std::string names[3] = {"coxa_joint", "femur_joint", "tibia_joint"};



float pwm_desired[18]={};
//float pwm_current[18]={};
float distance = 0;
int accel_X = 0;
int accel_Y = 0;
int accel_Z = 0;
int gyro_X = 0;
int gyro_Y = 0;
int gyro_Z = 0;

float vel_X=0, vel_Y=0, vel_Z=0;
float X=0, Y=0, Z=0;
float X_desired = 10, Y_desired = 0;
ros::Time t_stamp;

float ts = 0.01;
int time_loop_limit = 500; //(1000*ts)=10 sec

void pos_sensor_callback(gazebo_msgs::ModelStates msg){
  //Sensor
  X = msg.pose[1].position.x;
  Y = msg.pose[1].position.y;
  Z = msg.pose[1].position.z;

}


int main(int argc, char **argv)
{

	#ifdef DEBUG_H_INCLUDED
	ROS_INFO("Debugging active");
	#endif //DEBUG_H_INCLUDED

	ros::init(argc, argv, "Algorithm_node");

	srand (time(NULL));
  //srand(1000);

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
  ros::ServiceClient reset_simulation = n.serviceClient <std_srvs::Empty> ("/gazebo/reset_simulation");
  std_srvs::Empty reset_srvs;

	ros::Rate loop_rate(1/ts);

  std_msgs::Float64 global_pos_msgs;

	int time_loop = 0;

	float currentTime = 0;
	float InputVec[5] = {};//Input vector to the network
	float OutputVec[4] = {};//Output vector of the network
	float StepSize = 0.4; //Size of the possition change each second
	float Out_hlim = 1; //Up limit possition
	float Out_llim = -1; //Down limit positon
  bool end = 0;

	float Fitness = 0;

	//Initialize pub/subs

  //Initialize robot
  i = 0;
  for (int name=0; name<3; name++){
    for(int suf=0; suf<6; suf++){
      global_pos_msgs.data = 0;

      joint_pub[i].publish(global_pos_msgs);
      #ifdef DEBUG_H_INCLUDEDi
      ROS_INFO("%f",pwm_desired[i]);
      #endif //DEBUG_H_INCLUDED
      //position[1]++;
      i++;
    }
  }
  ros::Duration(1).sleep();

  reset_simulation.call(reset_srvs);

	//Initalization of the pool
	Pool Spidy_pool(5,4);


	#ifdef DEBUG_H_INCLUDED
	ROS_INFO("Pool created");
	#endif //DEBUG_H_INCLUDED

	Spidy_pool.initializePool();
    // Spidy_pool = customReadFile();
    //
    // Spidy_pool.currentGenome = 0;
    // Spidy_pool.currentSpecies = 0;

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

		memset(pwm_desired, 0, sizeof(pwm_desired));

    end = 0;

    time_loop_limit = 500 + floor(Spidy_pool.generation/10)*100;
    
		while((time_loop<time_loop_limit)&&(!end)){

			//TODO: Sensor read implementation

      InputVec[0]=pwm_desired[0];
      InputVec[1]=pwm_desired[1];
      InputVec[2]=pwm_desired[6];
      InputVec[3]=pwm_desired[7];

			currentTime = time_loop*ts;

      InputVec[4]=currentTime;

			Spidy_pool.evaluateCurrent(InputVec,OutputVec);


			//Process outputs
			for(int i=0;i<4;i++)
			{
				OutputVec[i] = OutputVec[i]*StepSize;
			}

      if((-pwm_desired[1]+pwm_desired[0])>-1){
        pwm_desired[0] += OutputVec[0]*ts;
      }
      if((-pwm_desired[1]+pwm_desired[0])>-1){
        if((-pwm_desired[2]+pwm_desired[1])>-1){
          pwm_desired[1] += -1*OutputVec[1]*ts;
        }
      }
      if((-pwm_desired[2]+pwm_desired[1])>-1){
        pwm_desired[2] += OutputVec[0]*ts;
      }



      if((-pwm_desired[4]+pwm_desired[3])>-1){
        pwm_desired[3] += OutputVec[1]*ts;
      }
      if((-pwm_desired[4]+pwm_desired[3])>-1){
        if((-pwm_desired[5]+pwm_desired[4])>-1){
          pwm_desired[4] += -1*OutputVec[0]*ts;
        }
      }
      if((-pwm_desired[5]+pwm_desired[4])>-1){
        pwm_desired[5] += OutputVec[1]*ts;
      }


      pwm_desired[6] += OutputVec[2]*ts;
      pwm_desired[12] += -1*OutputVec[2]*ts;
      pwm_desired[7] += OutputVec[3]*ts;
      pwm_desired[13] += -1*OutputVec[3]*ts;
      pwm_desired[8] += OutputVec[2]*ts;
      pwm_desired[14] += -1*OutputVec[2]*ts;
      pwm_desired[9] += OutputVec[3]*ts;
      pwm_desired[15] += -1*OutputVec[3]*ts;
      pwm_desired[10] += OutputVec[2]*ts;
      pwm_desired[16] += -1*OutputVec[2]*ts;
      pwm_desired[11] += OutputVec[3]*ts;
      pwm_desired[17] += -1*OutputVec[3]*ts;

      for(int i=0;i<18;i++)
      {
          if(pwm_desired[i]>Out_hlim)
        		pwm_desired[i]=Out_hlim;

  				if(pwm_desired[i]<Out_llim)
  					pwm_desired[i]=Out_llim;
      }


			//TODO: Send pwm(pwm_current)

      i = 0;
      for (int name=0; name<3; name++){
        for(int suf=0; suf<6; suf++){
          global_pos_msgs.data = pwm_desired[i];

          joint_pub[i].publish(global_pos_msgs);

          #ifdef DEBUG_H_INCLUDED
            ROS_INFO("Pwm: %f",pwm_desired[i]);
            if(i<12)
            ROS_INFO("Out val: %f",OutputVec[i]);
          #endif //DEBUG_H_INCLUDED
          //position[1]++;
          i++;
        }
      }


			ros::spinOnce();

      if(Z>0.25)
      {
        end =1;
      }

			loop_rate.sleep();
			time_loop++;
		}


    #ifdef DEBUG_H_INCLUDED
		ROS_INFO("Simulation ended");
		#endif //DEBUG_H_INCLUDED

    Fitness = 0;

		//Calculate fitness

    distance = sqrt(pow(Y-Y_desired,2)+pow(X-X_desired,2));


    if(!end)
    {
      if(distance>0)
      Fitness = 1/distance;
    }
		//Evaluation ended

    #ifdef DEBUG_ALGO_H_INCLUDED
    ROS_INFO("Distance: %f",distance);
    ROS_INFO("Fitness: %f",Fitness);
    #endif //DEBUG_ALGO_H_INCLUDED
		Spidy_pool.assignfitness(Fitness);

		//Next genome
		if(Spidy_pool.SpeciesVec[Spidy_pool.currentSpecies].GenomesVec.size()==Spidy_pool.currentGenome+1)
		{
    		if(Spidy_pool.SpeciesVec.size()==Spidy_pool.currentSpecies+1){
				#ifdef DEBUG_ALGO_H_INCLUDED
				ROS_INFO("Generating new generation...");
				#endif //DEBUG_ALGO_H_INCLUDED

        std::stringstream ss2;
        ss2 << Spidy_pool.generation;
        std::string generationstr = ss2.str();
        std::string textfile = "Generations/TestGen" + generationstr +".txt";

        customWriteFile(Spidy_pool,textfile);

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


    //Reset simulation


    i = 0;
    for (int name=0; name<3; name++){
      for(int suf=0; suf<6; suf++){
        global_pos_msgs.data = 0;

        joint_pub[i].publish(global_pos_msgs);
        #ifdef DEBUG_H_INCLUDED
        ROS_INFO("%f",pwm_desired[i]);
        #endif //DEBUG_H_INCLUDED
        //position[1]++;
        i++;
      }
    }

    ros::Duration(1).sleep();
    ros::spinOnce();

    //Call reset service

    reset_simulation.call(reset_srvs);

    #ifdef DEBUG_ALGO_H_INCLUDED
    ROS_INFO("Simulation reseted");
    #endif //DEBUG_ALGO_H_INCLUDED

    //ros::Duration(30).sleep();


    ros::spinOnce();


	}

  return 0;
}
