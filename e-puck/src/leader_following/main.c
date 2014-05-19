
#include <e_ad_conv.h>
#include <e_init_port.h>
#include <e_epuck_ports.h>
#include <e_uart_char.h>
#include <e_led.h>

#include <e_led.h>
#include <e_motors.h>
#include <e_agenda.h>

#include <stdio.h>
#include <stdlib.h>
#include <ircom.h>
#include <btcom.h>
#include <math.h>


typedef struct
{
    float angle;
    int sensor;
    float distance;
    long lastSeen;
} Neighbor;
Neighbor neighbor[5];

#define COM_CYCLE_SPEED 1000 // 6000
#define NEIGHBOUR_TTL 10000   // 30000
#define M_PI 3.141592654
#define MAX_SPIRAL 500
#define NUM_LEADER 0
#define LEADER_THRESHOLD 300
#define DISTANCE 3
#define ANGLE_THRESHOLD 30
#define DISTANCE_THRESHOLD 1


double leader_speed = 200;
double obstacleAvoidanceThreshold = 300.0;
double obstacleAvoidanceSpeed = 200.0;
int num;
int leader;
int spiral =100;
long int lastClock;
int selector;
float sensorDir[NB_IR_SENSORS] = {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865};

int getselector()
{
    return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

void obstacleAvoidance();

int main()
{   int i;
	long j;
    // init robot
    e_init_port();
    e_init_ad_scan();
    e_init_uart1();
    e_led_clear();	
    e_init_motors();
    e_start_agendas_processing();

	btcomSendString("-OK-\n");
    e_calibrate_ir(); 

    // initialize ircom and start reading
    ircomStart();
    ircomEnableContinuousListening();
	ircomListen();

	ircomResetTime();
    lastClock = ircomGetTime();
    // rely on selector to define the role

	selector = getselector();
	// The threshold of distance of leader and followers are different
	if (selector ==  NUM_LEADER)
	{  obstacleAvoidanceThreshold=LEADER_THRESHOLD;
	}
	//follow the robot with the id of your id - 1
	leader=selector-1;
	for (i=0 ;i<5 ;i++ )
	{ neighbor[i].distance= -1.0;
	  neighbor[i].angle=0.0;
	}

    // show selector choosen
    for (i = 0; i <= selector; i++)
    {
	e_led_clear();
	
	for(j = 0; j < 200000; j++)
	    asm("nop");
	
	e_set_led(i%8, 1);
	
	for(j = 0; j < 300000; j++)
	    asm("nop");

	e_led_clear();

	for(j = 0; j < 300000; j++)
	    asm("nop");
    }


    // activate obstacle avoidance
    e_activate_agenda(obstacleAvoidance, 500);

	if (selector < 5)
    {
		while (1)
		{   ircomDisableProximity();
			int messageReceived = 0;
			while(((ircomGetTime() - lastClock < COM_CYCLE_SPEED) || (ircomIsReceiving() == 1)))
			{			   
				IrcomMessage msg;
				ircomPopMessage(&msg);
				
				if (msg.error == 0)
				{   
					num = (int) msg.value;
					neighbor[num].angle = msg.direction;
					neighbor[num].distance = msg.distance;	  	
					neighbor[num].lastSeen = ircomGetTime();
					messageReceived = 1;
				}
				//else imsg.error == -1 -> no message available in the queue
			}
			e_set_body_led(messageReceived);
			lastClock = ircomGetTime();
			e_led_clear();
			ircomSend((long int)(selector));	   
			while (ircomSendDone() == 0);

			e_set_body_led(0);
			//wait to avoid interfere with proximity sensor
			for(j = 0; j < 1000; j++){
			   asm("nop");
			}	
		}
    }
    // no proper role defined...
    else 
    {
	while(1)
	{
	    e_led_clear();
	    
	    for(j = 0; j < 200000; j++)
		asm("nop");
	    
	    e_set_led(i, 1);
	    
	    for(j = 0; j < 300000; j++)
		asm("nop");
	    
	    i++;
	    i = i%8;
	}	
    }    
    
    ircomStop();
    return 0;
}

void obstacleAvoidance()
{    
    // check if an obstacle is perceived 
    
	double reading = 0.0;
    int obstaclePerceived = 0;
    int i=0;
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;
	double speed;
	double left=0,right=0;
	double delta;
	double e_dist;
	double e_angle;

    double x = 0.0, y = 0.0;
	ircomEnableProximity();
    for (i = 0; i < 8; i++)
    {
        reading = e_get_calibrated_prox(i);//e_get_prox(i);	
		// if signal above noise
		if(reading >= obstacleAvoidanceThreshold)
		{   obstaclePerceived = 1;
		    e_set_led(i,1);

			// compute direction to escape
			double signal = reading - obstacleAvoidanceThreshold;
			x += -cos(sensorDir[i]) * signal / 10.0;
			y += sin(sensorDir[i]) * signal / 10.0;
		}
    }
   
    // no obstacles to avoid, return immediately

    if (obstaclePerceived == 0)
    {
		// go straight forward
		// change movement direction
		if (selector!=NUM_LEADER)//im not leader
		{  	if (ircomGetTime() - neighbor[leader].lastSeen < NEIGHBOUR_TTL)
			{   //follow assigned leader
				e_dist = neighbor[leader].distance - 7 - DISTANCE;
				if (fabs(e_dist) < DISTANCE_THRESHOLD) e_dist = 0.0;
				speed = 50.0*e_dist;
				while(neighbor[leader].angle < -M_PI) neighbor[leader].angle += 2.0 * M_PI;
				while (neighbor[leader].angle >= M_PI) neighbor[leader].angle -= 2.0 * M_PI;
				e_angle = neighbor[leader].angle;

				if (fabs(e_angle) < ANGLE_THRESHOLD*M_PI/180.0) e_angle = 0.0;//ignore noise
				delta = 300.0*e_angle; 
				
				e_set_speed_left(speed - delta);
				e_set_speed_right(speed + delta);
			}
			else 
			{   //follower's searching bahavior -- go a spiral line
				e_set_speed_left(MAX_SPIRAL);
				spiral +=1;
				if (spiral >MAX_SPIRAL)
					spiral =100;
				e_set_speed_right(spiral);
				//e_set_speed_left(0);
				//e_set_speed_right(0);
			}


		}
		else// i am leader
		{
			if (ircomGetTime() - neighbor[NUM_LEADER+1].lastSeen < NEIGHBOUR_TTL)
			{ //no obstacle and i am followed by assigned follower
				//e_set_speed_left(leader_speed);
				//e_set_speed_right(leader_speed);
				e_set_speed_right(0);
				e_set_speed_left(0);
			}
			else//leader is not being followed act leader's waiting behavior 
			{   
				e_set_speed_left(0.0);
				e_set_speed_right(0.0);	
			}
		}
    }
	else{//avoid obstacle
		double desiredAngle = atan2 (y, x);
		// turn left
		if (desiredAngle >= 0.0)
		{
			leftSpeed  = cos(desiredAngle);
			rightSpeed = 1;
		}
		// turn right
		else
		{
			leftSpeed = 1;
			rightSpeed = cos(desiredAngle);
		}
	    
		// rescale values
		leftSpeed *= obstacleAvoidanceSpeed;
		rightSpeed *= obstacleAvoidanceSpeed;
	    
		// change movement direction
		e_set_speed_left(leftSpeed);
		e_set_speed_right(rightSpeed);
		
		 
	}
    // advertise obstacle avoidance in progress
    // return 1;
}
