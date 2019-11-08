#include <iostream>
#include <algorithm>  // for std::for_each
#include <stdio.h> //for printf
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <cmath>  // for pow
#include <math.h> // for atan2
#include <Stim300RevG.hpp>
#include <Eigen/Geometry> 

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
constexpr double PI{3.14159265358979323846};




/////////////// - inclinometer code
struct Quaternion
{
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles FromQuaternionToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}


Quaternion FromRPYToQuaternion(EulerAngles angles) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}

// Global Quaternion
Quaternion globqat;

void quatFromEkfCallback(const sensor_msgs::Imu::ConstPtr& message)
{
    globqat.w = message->orientation.w;
    globqat.x = message->orientation.x;
    globqat.y = message->orientation.y;
    globqat.z = message->orientation.z;
}


/////// - inclinometer code 







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

    // Inclinometer -- 

    ros::Subscriber EKFsubscriber = node.subscribe("quat/ekf",1000,quatFromEkfCallback);

    // inclinometer  -- 
    ros::Publisher imuSensorPublisher = node.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);
    ros::Rate loop_rate(sampleRate+1);

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
            
            double inclinationX{0};
            double inclinationY{0};
            double inclinationZ{0};
            Quaternion q;
            EulerAngles RPY;
            EulerAngles yawFromEKF;

            


            // Get inclination data and convert to roll and pitch

            inclinationX = myDriverRevG.getInclData()[0];
            inclinationY = myDriverRevG.getInclData()[1];
            inclinationZ = myDriverRevG.getInclData()[2];

           // cout<<inclinationX<<endl;
           // cout<<inclinationY<<endl;
           // cout<<inclinationZ<<endl;

            yawFromEKF = FromQuaternionToEulerAngles(globqat);
            

            RPY.roll = atan2(inclinationY,inclinationZ)*(180.0/PI);
            RPY.pitch = atan2(-inclinationX,sqrt(pow(inclinationY,2)+pow(inclinationZ,2)))*(180.0/PI);
            RPY.yaw = yawFromEKF.yaw;

            q = FromRPYToQuaternion(RPY);

    	    cout<<"roll: "<<RPY.roll<<endl;
            cout<<"pitch: "<<RPY.pitch<<endl;
            cout<<"yaw_from_ekf"<< RPY.yaw<<endl;

            //myDriverRevG.printInfo();
            stim300msg.orientation_covariance[0] = 0.2; 
            stim300msg.orientation_covariance[4] = 0.2;
            stim300msg.orientation_covariance[8] = 0.2;
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

            stim300msg.orientation.x = q.x;
            stim300msg.orientation.y = q.y;
            stim300msg.orientation.z = q.z;
            stim300msg.orientation.w = q.w;

            imuSensorPublisher.publish(stim300msg);
            ++countMessages;

        }

        loop_rate.sleep();

        ros::spinOnce();

    }


    myDriverRevG.close();
	
    return 0;
}
