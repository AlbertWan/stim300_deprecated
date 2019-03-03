#include <iostream>
#include <algorithm>  // for std::for_each
#include <stdio.h> //for printf
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <cmath>  // for pow
#include <Stim300RevD.hpp>
#include <Stim300RevB.hpp>
#include <Stim300RevG.hpp>


// For ROS to work properly 

#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

//////////////



using namespace std;


///////////////

const double averageAllanVarianceOfGyro{2*4.6*pow(10,-4)};
const double averageAllanVarianceOfAcc{2*5.2*pow(10,-3)};
const int sampleRate{125};



double getAllanVarianceToVarianceOfGyro(){
    double variance{0};

    variance=sampleRate*pow(averageAllanVarianceOfGyro,2);

    return variance;
}

double getAllanVarianceToVarianceOfAcc(){
    double variance{0};

    variance=sampleRate*pow(averageAllanVarianceOfAcc,2);

    return variance;
}

////////////////

/**
* SIGNAL      ID   DEFAULT  DESCRIPTION
* ===============================================
*	SIGHUP 		1    Termin.  Hang up on controlling terminal
*  SIGINT			2    Termin.  Interrupt. Generated when we enter CNRTL-C and it is delivered
*								to all processes/threads associated to the current terminal. If generated
*								with kill, it is delivered to only one process/thread.
*  SIGQUIT      3    Core     Generated when at terminal we enter CNRTL-\
*  SIGILL         4    Core     Generated when we executed an illegal instruction
*  SIGTRAP     5    Core     Trace trap (not reset when caught)
*  SIGABRT     6    Core     Generated by the abort function
*  SIGFPE       8    Core     Floating Point error
*  SIGKILL      9    Termin.  Termination (can't catch, block, ignore)
*  SIGBUS     10    Core     Generated in case of hardware fault
*  SIGSEGV   11    Core     Generated in case of illegal address
*  SIGSYS      12    Core     Generated when we use a bad argument in a system service call
*  SIGPIPE     13    Termin.  Generated when writing to a pipe or a socket while no process is reading at other end
*  SIGALRM   14    Termin.  Generated by clock when alarm expires
*  SIGTERM   15    Termin.  Software termination signal
*  SIGURG     16    Ignore   Urgent condition on IO channel
*  SIGCHLD   20    Ignore   A child process has terminated or stopped
*  SIGTTIN     21    Stop     Generated when a backgorund process reads from terminal
*  SIGTTOUT  22    Stop     Generated when a background process writes to terminal
*  SIGXCPU    24    Discard  CPU time has expired
*  SIGUSR1    30    Termin.  User defiled signal 1
*  SIGUSR2    31    Termin.  User defined signal 2
*
* @author Javier Hidalgo Carrio .
*/

/**
* @brief POINTER TO HANDLING FUNCTION
*
* Defines for the signaling library. Definitiod of data type for pointer to handling function
*/
typedef void (* handling_t) (int sig, void *data);

/**
* @brief GLOBAL ARRAY.
*
*  Two arrays:
*  		handling_t htab: one for pointer of function (the handling function)
* 		data: array of handling function parameters. 
*/

static handling_t htab [20] ;
static void *data[20];

/**
* @brief Function that set a generic handler for a signal.
* 
* @author Javier Hidalgo Carrio.
* @param[in] sig integer for the number of signal to be handled
*
* @return Void
*
*/
static void generic_handling (int sig)
{

   htab[sig](sig,data[sig]);

}

/**
* @brief Function that set a defined handler for a signal.
* 
* @author Javier Hidalgo Carrio.
* @param[in] sig integer for the number of signal to be handled
* @param[in] pfunc pointer of the handling function
* @param[in] values pointer of void, the parameter of the handling function.
*
* @return OK is everything all right. ERROR on other cases
*
*/
bool signal_set (int sig,handling_t pfunc,void *values)
{

	htab[sig] = pfunc;
	data[sig] = values;
	if ((signal (sig,generic_handling))==SIG_ERR)
	 return false;
	return true;
}


/**
* @brief The Function catchs the signal introduced in the parameter of this function
* 
* The function associates the signals with the handling function to be handled when
* the process would receives such signal from the operating system.
* The handling function should have two parameters when defined in the main program.
* one parameter (integer) for the signal number, another for the pointer to void (values)
* Normally the last one is a pointer to a structure.
*
* @author Javier Hidalgo Carrio.
* @param[in] sig_1 integer for the number of signal 1 to be handled
* @param[in] sig_2 integer for the number of signal 2 to be handled
* @param[in] sig_3 integer for the number of signal 3 to be handled
* @param[in] sig_4 integer for the number of signal 4 to be handled
* @param[in] pfunc pointer to the handling function
* @param[in] values pointer to void, the parameter of the handling function.
*
* @return OK is everything all right. ERROR in other cases
*
*/
bool signal_catcher (int sig_1,int sig_2,int sig_3, int sig_4, handling_t pfunc, void *values)
{
	bool status=true;

	status=signal_set (sig_1,pfunc,values);

	if (status)

	status=signal_set (sig_2,pfunc,values);

	if (status)

	status=signal_set (sig_3,pfunc,values);

	if (status)

	status=signal_set (sig_4,pfunc,values);
	
	return status;

}

/**
 * @brief Function for handling system signals.
 * 
 * This function is the handler when the catched
 * system signal happen.
 *
 * @author Javier Hidalgo Carrio.
 *
 * @param[in] sig integer of the catched signal
 * @param[in] values pointer of void (pointer of the IMU structure after casting in the function)
 *
 * @return void
 *
 */
void signal_terminator (int sig, void *values)
{
	bool status = true;
	imu_stim300::Stim300Base * pdriver;

	pdriver = (imu_stim300::Stim300Base *) values;

	if (sig == SIGINT)
		std::cout<< "\nSIGINT: Terminal interrupt\n";
	else if (sig == SIGTERM)
		std::cout<< "\nSIGTERM: Termination\n";
	else if (sig == SIGHUP)
		std::cout<< "\nSIGHUP: Hangup\n";
	else std::cout<<"\nSIGSEGV: Segmentation\n";


	
        pdriver->close();
        std::cout<<"serial port closed correctly.\n";
	
  	exit (status);
	
}


int main(int argc , char **argv)
{
   
	imu_stim300::Stim300RevG myDriverRevG;

    handling_t pfunc;
	
    if (argc<2)
    {
	printf( "Usage: imu_stim300_bin <device>\n");
	return 0;
    }

        /* Function signal handling */
    pfunc = signal_terminator;

    /* Catching system signals */
    signal_catcher (SIGHUP,SIGINT,SIGTERM, SIGSEGV, pfunc, &myDriverRevG);


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


    // ROS PART ---------------------------------------------

      // Initlize node


    ros::init(argc, argv,  "stim300");

    ros::NodeHandle node;


    ros::Publisher imuSensorPublisher = node.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

    ros::Rate loop_rate(sampleRate);

    //int i{0};

    int differenceInDataGram{0};
    int countMessages{0};

    ROS_INFO("Publishing sensor data from IMU");
    while(ros::ok()){
        
        
        sensor_msgs::Imu stim300msg;

        myDriverRevG.processPacket();
        stim300msg.header.stamp = ros::Time::now();
        stim300msg.header.frame_id = "imu_frame";
        differenceInDataGram = myDriverRevG.getDatagramCounterDiff();

        
        if (myDriverRevG.getStatus() == false)
        {
            ROS_WARN("stim300 internal error ");
        }

         if (myDriverRevG.getChecksumStatus() == false)
        {
            ROS_WARN("stim300 CRC error ");
        }

        if (differenceInDataGram != 16)
        {
			std::cout << "Current Datagram counter diff : " << differenceInDataGram << "\n\n";
            
		}

        
       
        /* Covariance matrix
        variance_x² 0 0
        0 variance_y² 0 
        0 0 variance_z²
        */

        stim300msg.orientation_covariance[0] = -1;
        stim300msg.angular_velocity_covariance[0] = getAllanVarianceToVarianceOfGyro();
        stim300msg.angular_velocity_covariance[4] = getAllanVarianceToVarianceOfGyro();
        stim300msg.angular_velocity_covariance[8] = getAllanVarianceToVarianceOfGyro();                                  
        stim300msg.linear_acceleration_covariance[0] = getAllanVarianceToVarianceOfAcc();
        stim300msg.linear_acceleration_covariance[4] = getAllanVarianceToVarianceOfAcc();
        stim300msg.linear_acceleration_covariance[8] = getAllanVarianceToVarianceOfAcc();


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

        ros::spinOnce();

        loop_rate.sleep();

        ++countMessages;

    }

/*
    while (i < 1000)
    {
        i++;

        
        //usleep(9800);


        myDriverRevG.processPacket();


        myDriverRevG.printInfo();

        //std::cout<<"Datagram counter diff: "<< myDriverRevG.getDatagramCounterDiff()<<"\n";
        //std::cout<<"Gyro: " <<myDriverRevG.getGyroData()[0] <<"\n";
        diff = myDriverRevG.getDatagramCounterDiff();


        if (diff != 16)
        {
			std::cout << "Current Datagram counter diff : " << diff << "\n\n";
		}
    }

	
// 	usleep (2000);
// 	if (i == 800)
// 	{
// 	    std::cout<<"RESET IN NORMAL MODE\n";
// 	    myDriverRevD.fullReset();
// 	}
//	
// 	if (i == 1600)
// 	{
// 	    std::cout<<"ENTER IN SERVICE MODE\n";
// 	    myDriverRevD.enterServiceMode();
//
// 	}
//	
// 	if (i == 1604)
// 	{
// 	    std::cout<<"EXIT SERVICE MODE\n";
// 	    myDriverRevD.fullReset();
// 	}
//	
// 	if (i == 3200)
// 	{
// 	    std::cout<<"ACC TO INCREMENTAL VELOCITY\n";
// 	    myDriverRevD.setAcctoIncrementalVelocity();
// 	}
//	
//	std::cout<<"I: "<<i<<"\n";
//	i++;
	
//    } 
*/

    myDriverRevG.close();
	
    return 0;
}