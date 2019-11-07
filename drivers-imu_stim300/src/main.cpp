#include <iostream>
#include <algorithm>  // for std::for_each
#include <stdio.h> //for printf
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <cmath>  // for pow
#include <math.h> // for atan2
#include <Stim300RevG.hpp>


// For ROS to work properly 

#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

//////////////



using namespace std;


///////////////
constexpr int defaultSampleRate{125}; // In hz
constexpr double averageAllanVarianceOfGyro{0.0001*2*4.6*pow(10,-4)};
constexpr double averageAllanVarianceOfAcc{100*2*5.2*pow(10,-3)};









int main(int argc , char **argv)
{
   
	imu_stim300::Stim300RevG myDriverRevG;

    //handling_t pfunc;
	
    if (argc<2)
    {
	printf( "Usage: imu_stim300_bin <device>\n");
	return 0;
    }

    

  
    // ROS PART ---------------------------------------------

      // Initlize node


    ros::init(argc, argv,  "stim300");

    ros::NodeHandle node;

    double stanardDeivationOfGyro{0};
    double stanardDeviationOfAcc{0};
    double varianceOfGyro{0};
    double varianceOfAcc{0};
    int sampleRate{0};




    node.param("stanard_deviation_of_gyro",stanardDeivationOfGyro,averageAllanVarianceOfGyro);
    node.param("stanard_deviation_of_acc",stanardDeviationOfAcc,averageAllanVarianceOfAcc);
    node.param("sample_rate", sampleRate , defaultSampleRate );


    varianceOfGyro=sampleRate*pow(stanardDeivationOfGyro,2);
    varianceOfAcc=sampleRate*pow(stanardDeviationOfAcc,2);
    


    myDriverRevG.welcome();

    myDriverRevG.setBaudrate(iodrivers_base::Driver::SERIAL_921600);
    myDriverRevG.setFrequency(sampleRate);
    myDriverRevG.setPackageTimeout(0.1);


    if (!myDriverRevG.open(argv[1]))
    {
	cerr << "cannot open device: " << argv[1] << endl;
	perror("errno is");
	return 1;
    }
    usleep(98000);

    ros::Publisher imuSensorPublisher = node.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

    ros::Rate loop_rate(sampleRate+1);

    //int i{0};

    int differenceInDataGram{0};
    int countMessages{0};

    ROS_INFO("Publishing sensor data from IMU");
    while(ros::ok()){
        
        
        sensor_msgs::Imu stim300msg;

        myDriverRevG.processPacket();
        stim300msg.header.stamp = ros::Time::now();
        stim300msg.header.frame_id = "imu_0";
        differenceInDataGram = myDriverRevG.getDatagramCounterDiff();

        
        if (myDriverRevG.getStatus() == false)
        {
            ROS_WARN("stim300 internal error ");
        }

         if (myDriverRevG.getChecksumStatus() == false)
        {
            ROS_WARN("stim300 CRC error ");
        }
        else
        {  
            double roll{0};
            double pitch{0};
            double inclinationX{0};
            double inclinationY{0};
            double inclinationZ{0};


            // Get inclination data and convert to roll and pitch

            inclinationX = myDriverRevG.getInclData()[0];
            inclinationY = myDriverRevG.getInclData()[1];
            inclinationZ = myDriverRevG.getInclData()[2];

           // cout<<inclinationX<<endl;
           // cout<<inclinationY<<endl;
           // cout<<inclinationZ<<endl;


            roll = atan2(inclinationY,inclinationZ)*(180.0/3.14159265358979323846);
            pitch = atan2(-inclinationX,sqrt(pow(inclinationY,2)+pow(inclinationZ,2)))*(180.0/3.14159265358979323846);


    	    cout<<"roll: "<<roll<<endl;
            cout<<"pitch: "<<pitch<<endl;
            //myDriverRevG.printInfo();
             
            stim300msg.orientation_covariance[0] = -1;
            stim300msg.angular_velocity_covariance[0] = varianceOfGyro;
            stim300msg.angular_velocity_covariance[4] = varianceOfGyro;
            stim300msg.angular_velocity_covariance[8] = varianceOfGyro;                                  
            stim300msg.linear_acceleration_covariance[0] = varianceOfAcc;
            stim300msg.linear_acceleration_covariance[4] = varianceOfAcc;
            stim300msg.linear_acceleration_covariance[8] = varianceOfAcc;


            //
            // Place sensor data from IMU to message

            stim300msg.linear_acceleration.x = myDriverRevG.getAccData()[0];
            stim300msg.linear_acceleration.y = myDriverRevG.getAccData()[1];
            stim300msg.linear_acceleration.z = myDriverRevG.getAccData()[2];

            stim300msg.angular_velocity.x = myDriverRevG.getGyroData()[0];
            stim300msg.angular_velocity.y = myDriverRevG.getGyroData()[1];
            stim300msg.angular_velocity.z = myDriverRevG.getGyroData()[2];

            stim300msg.orientation.x = 0;
            stim300msg.orientation.y = 0;
            stim300msg.orientation.z = 0;

            imuSensorPublisher.publish(stim300msg);
            ++countMessages;

        }

        loop_rate.sleep();

        ros::spinOnce();

    }


    myDriverRevG.close();
	
    return 0;
}
