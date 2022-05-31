

#ifndef LINEFOLLOWERBOT_H_
#define LINEFOLLOWERBOT_H_


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "std_types.h"
#include "common_macros.h"
#include <stdlib.h>


#define DC_MOTOR_MOVING_STOP	0
#define DC_MOTOR_MOVING_FORWARD	1
#define DC_MOTOR_MOVING_RIGHT	2
#define DC_MOTOR_MOVING_LEFT	3


/*******************************************************************************
 *                      Functions Prototypes                                   *
 *******************************************************************************/

/*
 * Description : Function responsible for initializing Timer0
 */

void Timer0_PWM_MODE_Init(uint8 dutyCycle);

/*
 * Description :
 * Function responsible for initialize the ADC driver.
 */

void ADC_Init(void);

/*
 * Description :
 * Function responsible for reading the ADC value.
 */

uint16 ADC_readChannel(uint8 channelNumber);



/*
 * Description :
 * Function responsible for initialize the DCmotor.
 */
void DCmotor_Init(void);

/*
 * Description :
 * Function responsible for setting the DCmotor's operation mode.
 */
void DCmotor_set_Operation_DirectionMode(uint8 motorDirection);


#include <stdlib.h>


#define States_count 8		/*Variable to store the Number of states in the learning problem*/

#define Actions_count 4		/*Variable to store the Number of possible actions to be taken by the robot*/

/*2D Array of size 8*4 to store the values of Q calculated for each (S,a) pair, initialized with all 0's */
float32 Q[States_count][Actions_count]={{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};

/*2D Array of size 8*4 to store the values of Rewards calculated for each (S,a) pair, initialized with all 0's */
 uint16 Rewards[States_count][Actions_count]={{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};


#define False 0
#define True  1
#define training_Count 100


/******************************Variables**********************************/

const float32 ALPHA = 0.8;    	/*Learning rate */

const float32 GAMMA = 0.8;    	/*Discount Factor*/

uint8 Robot_CurrentState;    /*Variable to store the Current state indicator of the robot */

uint8 Robot_Action_ToTake;

uint8 Next_State;			/*Variable to store the next state of the robot*/

float32 Q_new,Q_old,Q_Max;	/*Variables to store values of Q*/





/*Array of size 4 to store the values representing the actions performed by the robot where:
 * (0:Forward, 1:Right , 2:Left, 3:Stop)*/
uint8 Actions[4] = {0, 1, 2, 3} ;


void Update_Q(int next_state, int Action,  int Reward);
void MoveForward(void);
void TurnLeft(void);
void TurnRight(void);
void Stop(void);
int Get_stateNumber(void);
void Start_Robot_Trainig(void);
float Get_max_Q(int stateNum);
int Get_max_Q_Index(int stateNum);
void Start_Robot_Testing(void);

#endif /* LINEFOLLOWERBOT_H_ */
