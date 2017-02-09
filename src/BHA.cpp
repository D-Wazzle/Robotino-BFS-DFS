
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include "rec/robotino/api2/all.h"

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
        
        char x;
        while(true){
            std::cout << "please pose the arm in the position you would like it to stay in. type 'y' to accept pose  " << std::endl;
            std::cin>> x;
            if(x=='y'){
                cbha.stringPots( targetValues );
            }
            std::cout << "here is your pose." << std::endl;
            for( unsigned int i = 0; i < 6; ++i )
            {
                std::cout << "String pot " << i  << ": " << targetValues[i] << std::endl;
            }
            std::cout << "accept y/n" << std::endl;
            std::cin>> x;
            if(x=='y'){
                break;
            }
        }
        
        cbha.setCompressorsEnabled( true );
        
	}
}

void drive()
{
    float pressures[8] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    float values[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    //const float maxPressure = 1.5f;
    float stepSize=.5;
    //float lastTime=com.msecsElapsed();
    float lowerAvg;
    float upperAvg;

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
        
        cbha.setPressures( pressures );

        
        
        // list values
        
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

        
        // syncronize and delay
        //lastTime=com.msecsElapsed();
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

	std::string hostname = "172.26.1.1";
 
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
