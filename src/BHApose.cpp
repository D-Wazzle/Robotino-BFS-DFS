
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <vector>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>
#include "rec/robotino/api2/all.h"
#include <std_msgs/Bool.h>

using namespace rec::robotino::api2;

float _rect[2]= {0,0};

bool _run = true;


class MyCom : public Com
{
public:
	MyCom()
		: Com( "rect" )
	{
	}

	void errorEvent( const char* errorString )
	{
		std::cerr << "Error: " << errorString << std::endl;
	}

	void connectedEvent()
	{
		std::cout << "Connected." << std::endl;
	}

	void connectionClosedEvent()
	{
		std::cout << "Connection closed." << std::endl;
	}

	void logEvent( const char* message, int level )
	{
		std::cout << message << std::endl;
	}

	void pingEvent( float timeMs )
	{
		//std::cout << "Ping: " << timeMs << "ms" << std::endl;
	}
};

class MyBumper : public Bumper
{
public:
	MyBumper()
		: bumped( false )
	{
	}

	void bumperEvent( bool hasContact )
	{
		bumped |= hasContact;
		std::cout << "Bumper has " << ( hasContact ? "contact" : "no contact") << std::endl;
	}

	bool bumped;
};


MyCom com;
OmniDrive omniDrive;
MyBumper bumper;
CompactBHA cbha;

float targetValues[6] = { 0.591398, 0.522972, 0.532747, 0.796676, 0.821114, 0.904203};

bool gripperClosed=false;

// our calback funciton to retrieve the pose
void poseCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
 
	int i = 0;
	// print all the remaining numbers
	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		targetValues[i] = *it;
		i++;
		std::cout<<targetValues[i]<<std::endl;
	}
 
	return;
}


// gripper calback function.  Currenty just closes the gripper when the funciton is activated but it takes a string
// so that it can be extended later on.
void gripperCallback(const std_msgs::Bool& msg)
{
  if(msg.data){
    ROS_INFO("Close Gripper");
    gripperClosed=true;
  }
  else{
    ROS_INFO("Open Gripper");
    gripperClosed=false;
  }
}

void init( const std::string& hostname )
{
	// Initialize the actors

	// Connect
	std::cout << "Connecting... ";
	com.setAddress( hostname.c_str() );

	com.connectToServer( true );

	if( false == com.isConnected() )
	{
		std::cout << std::endl << "Could not connect to " << com.address() << std::endl;

		rec::robotino::api2::shutdown();
		exit( 1 );
	}
	else
	{
	std::cout << "success" << std::endl;
        
       
        cbha.setCompressorsEnabled( true );
        
	}
}

void drive()
{
    float pressures[8] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    float values[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    const float maxPressure = 1.5f;
    float stepSize=.5;
    float lastTime=com.msecsElapsed();
    float lowerAvg;
    float upperAvg;
    
    float errors[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float tolerance=.02;
    
    bool inTolerance=false;
    
    ros::NodeHandle n;
    ros::Subscriber bha_sub = n.subscribe("/bha_pose", 100, poseCallback);
    
    ros::Subscriber sub = n.subscribe("/bha_gripper", 100, gripperCallback);


	while( com.isConnected() && false == bumper.value() && _run )
	{
        // update the string pot values
        cbha.stringPots( values );
        
        // move toward target
        
        pressures[0]=pressures[0]+(-values[0]+targetValues[0])*stepSize+(values[1]-targetValues[1])*stepSize+(values[2]-targetValues[2])*stepSize;
        pressures[1]=pressures[1]+(values[0]-targetValues[0])*stepSize+(-values[1]+targetValues[1])*stepSize+(values[2]-targetValues[2])*stepSize;
        pressures[2]=pressures[2]+(values[0]-targetValues[0])*stepSize+(values[1]-targetValues[1])*stepSize+(-values[2]+targetValues[2])*stepSize;
        
        lowerAvg=(pressures[0]+pressures[1]+pressures[2])/3;
        
        pressures[0]-=lowerAvg;
        pressures[1]-=lowerAvg;
        pressures[2]-=lowerAvg;
            
        pressures[3]=pressures[3]+(-values[3]+targetValues[3])*stepSize+(values[4]-targetValues[4])*stepSize+(values[5]-targetValues[5])*stepSize;
        pressures[4]=pressures[4]+(values[3]-targetValues[3])*stepSize+(-values[4]+targetValues[4])*stepSize+(values[5]-targetValues[5])*stepSize;
        pressures[5]=pressures[5]+(values[3]-targetValues[3])*stepSize+(values[4]-targetValues[4])*stepSize+(-values[5]+targetValues[5])*stepSize;
 
        upperAvg=(pressures[3]+pressures[4]+pressures[5])/3;
        
        pressures[3]-=upperAvg;
        pressures[4]-=upperAvg;
        pressures[5]-=upperAvg;
	
	pressures[7]=10;
        
        cbha.setPressures( pressures );

        
        
	for( unsigned int i = 0; i < 6; ++i ){
            errors[i]=targetValues[i]-values[i];
            std::cout<< errors[i] << std::endl;
            if(tolerance > std::abs(errors[i])   ){
                
                //std::cout << "in tolerance: " << i <<  std::endl;
            }
            else{
                break;
            }
            
            if(i==5){
                 inTolerance=true;
		 //std::cout << "all in tolerance" << std::endl;
            }
            else inTolerance=false;
            
        }
        if(inTolerance) std::cout << "all in tolerance" << std::endl;
	else std::cout << "not in tolerance" << std::endl;

	cbha.setGripperValve1( gripperClosed );
        cbha.setGripperValve2( gripperClosed );

	
        // list values
        /*
        std::cout << std::endl;
        
        std::cout << "avg: " << lowerAvg << " bar" << std::endl;
        
        std::cout << std::endl;
        
        for( unsigned int i = 0; i < 8; ++i )
        {
            std::cout << "B" << i << ": " << pressures[i] << " bar" << std::endl;
        }
        
        std::cout << std::endl;
        
        for( unsigned int i = 0; i < 6; ++i )
        {
            std::cout << "String pot " << i  << ": " << values[i] << std::endl;
        }

        */
        // syncronize and delay
        
        ros::spinOnce();
        
        lastTime=com.msecsElapsed();
		com.processEvents();
		rec::robotino::api2::msleep( 100 );
	}
	
	
    
    
}

void destroy()
{
	com.disconnectFromServer();
}

int main( int argc, char **argv )
{
	ros::init(argc, argv, "BHApose");
	std::string hostname = "192.168.0.2";
 
	if( argc > 1 )
	{
		hostname = argv[1];
	}

	try
	{
		init( hostname );
		drive();
		destroy();
	}
	catch( const rec::robotino::api2::RobotinoException& e )
	{
		std::cerr << "Com Error: " << e.what() << std::endl;
	}
	catch( const std::exception& e )
	{
		std::cerr << "Error: " << e.what() << std::endl;
	}
	catch( ... )
	{
		std::cerr << "Unknow Error" << std::endl;
	}

	rec::robotino::api2::shutdown();
}
