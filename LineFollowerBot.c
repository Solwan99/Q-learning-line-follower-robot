#include "LineFollowerBot.h"


uint8 left_sensor, center_sensor, right_sensor;
/*Function to initialize Timer0 with fast PWM mode operation, having a duty cycle as a function parameter*/

/*Function to initialize Timer0 with fast PWM mode operation, having a duty cycle as a function parameter*/
void Timer0_PWM_MODE_Init(uint8 dutyCycle)
{
	/*Initialize the timer's counter value in the timer counter register TCNT0=0*/
	TCNT0=0;
	/*The Output Compare Register OCR0 contains the duty cycle value received as a function input parameter from the ADC converted
	 * value of the analog variable voltage reading taken from the variable resistance that is continuously compared with
	 *  the counter value*/
	_SFR_IO8(0x27)=dutyCycle;

	/*Configure the timer operation as:Waveform Generation Mode Fast PWM mode
	 * by writing the bits WGM01=1 && WGM00=1 in the timer0 control register TCCR0*
	SET_BIT(TCCR0,WGM01);
	SET_BIT(TCCR0,WGM00);
*/
	/*Configure the output compare mode to be Clear OC0 on compare match, set OC0 at BOTTOM, (non-inverting mode)
	 *by writing the bits COM01=1 && COM00=0 in the timer0 control register TCCR0 *
	SET_BIT(TCCR0,COM01);
	CLEAR_BIT(TCCR0,COM00);
*/
	/*Configure the timer's clock source as an internal source with frequency preScalar=F_CPU/8
	 *by writing the bits CS02=0 CS01=1 CS00=0 in the timer0 control register TCCR0
	CLEAR_BIT(TCCR0,CS00);
	SET_BIT(TCCR0,CS01);
	CLEAR_BIT(TCCR0,CS02);
	*/

	_SFR_IO8(0x24)|= (1<<7)|(1<<WGM01)|(1<<WGM00);
	_SFR_IO8(0x25)|=(1<<3);
}

void ADC_Init(void)
{
	/*select the voltage reference for the ADC by setting the Reference Selection Bits REFS1=0 REFS0=0
	 * 1-to work with voltage: AREF, Internal Vref turned off
	 * 2-MUX4:0: Analog Channel and Gain Selection Bits to work with channel ADC0>> MUX4:0=0000*/
	ADMUX=0;
	/*Configure the ADC control and status register ADCSRA
	 * 1-Writing the ADEN bit to one to enable the ADC
	 * 2-Disable the ADC interrupt by clearing the bit ADIE: ADC Interrupt Enable
	 * 3-ADC Pre-scalar Select Bits ADPS2=0 ADPS1=1 ADPS0=1
	 *  to provide the ADC with with a clock source of frequency=F_CPU/8*/
	ADCSRA|= (1<<ADEN) | (1<<ADPS1) | (1<<ADPS0);
}

uint16 ADC_readChannel(uint8 channelNumber){
	/*Ensure that the channel number is in the range 0 - 7, because those are the only channels existing in the AVR ATmega16 MCU*/


	/*first 5 bits in ADMUX register MUX4:0: Analog Channel and Gain Selection Bits to work with channelNumber*/
	ADMUX=(ADMUX&0b11100000)|(channelNumber&00011111);

	/*Start conversion set the bit ADSC to 1*/
	ADCSRA|=(1<<ADSC);

	/*wait for conversion to complete and the ADC interrupt flag bit ADIF becomes 1*/
	/*؛Polling*/
	while(BIT_IS_CLEAR(ADCSRA,ADIF));

	/*When conversion has completed, clear the flag bit by writing 1 in it*/
	ADCSRA|=(1<<ADIF);

	/*Return the data that has been read by the ADC*/
	return ADC;
}


void DCmotor_Init(void)
{
	/*Make PORTD pins 0,1 as output pins for the motor*/
	DDRD|=(1<<PD6)|(1<<PD5)|(1<<PD3)|(1<<PD4);

}
void DCmotor_set_Operation_DirectionMode(uint8 motorDirection)
{

	switch (motorDirection)
	{
		/*Stop the motor's motion*/
	case DC_MOTOR_MOVING_STOP:
		PORTD|=(1<<PD6);
		PORTD|=(1<<PD4);
		PORTD&= ~(1<<PD5);
		PORTD&= ~(1<<PD3);
		Timer0_PWM_MODE_Init(0);
		break;

	/*Rotate the motor in a Clock-wise manner*/
	case DC_MOTOR_MOVING_FORWARD:

		PORTD|=(1<<PD6);
		PORTD|=(1<<PD3);
		PORTD&= ~(1<<PD5);
		PORTD&= ~(1<<PD4);
		Timer0_PWM_MODE_Init(255);
		break;

	/*Rotate the motor RIGHT*/
	case DC_MOTOR_MOVING_RIGHT:
		PORTD|=(1<<PD5);
		PORTD|=(1<<PD3);
		PORTD&= ~(1<<PD4);
		PORTD&= ~(1<<PD6);
		Timer0_PWM_MODE_Init(255);
		break;

	case DC_MOTOR_MOVING_LEFT:
		PORTD|=(1<<PD6);
		PORTD|=(1<<PD4);
		PORTD&= ~(1<<PD5);
		PORTD&= ~(1<<PD3);
		Timer0_PWM_MODE_Init(255);
		break;
	}

}


/*=====================================Q-learning Algorithm Functions================================================*/

/*Function to update the Q-Array and Q values according to the Q-learning rules.
 * The update is continuous until the main loop ends. at each call back from this function,
 * the Q-Array is updated with different values while the robot trains itself.
 * Where the greater the value of Q, the greater the importance of this particular action to be taken
 * at this particular state.
 * >>> Q_old is the stored Q value
 * >>> Q_new is the new value calculated for Q with the learning algorithm formula(BELLMAN EQUATION)
 * >>> Q_Max is the maximum Q value stored in the Q-Array
 *  */
void Update_Q(int next_state, int Action,  int Reward)
{
	  Q_old = Q[Robot_CurrentState][Action];
	  Q_Max = Get_max_Q( Get_stateNumber() );
	  Q_new = ( ( 1-ALPHA ) * Q_old ) + ( ALPHA * ( Reward + ( GAMMA * Q_Max ) ) );
	  Q[Robot_CurrentState][Action] = Q_new;
}

/*===============================Robot's functions=======================================*/

/*Function to adjust the robot's motion direction to move it Forward by controlling the DC motors*/
void MoveForward(void)
{
	DCmotor_set_Operation_DirectionMode(DC_MOTOR_MOVING_FORWARD);
}

/*Function to adjust the robot's motion direction to make it Turn Right by controlling the DC motors*/
void TurnLeft(void)
{
	DCmotor_set_Operation_DirectionMode(DC_MOTOR_MOVING_LEFT);
}

/*Function to adjust the robot's motion direction to make it Turn Right by controlling the DC motors*/
void TurnRight(void)
{
	DCmotor_set_Operation_DirectionMode(DC_MOTOR_MOVING_RIGHT);
}

/*Function to adjust the robot's motion direction to make it Stop by controlling the DC motors*/
void Stop(void)
{
	DCmotor_set_Operation_DirectionMode(DC_MOTOR_MOVING_STOP);
}

int Get_stateNumber(void){

	int state_number;


	center_sensor=ADC_readChannel(2);
	right_sensor=ADC_readChannel(0);
	left_sensor=ADC_readChannel(4);

	if ( (left_sensor==0) && (center_sensor==0) && (right_sensor==0) )
	{
		state_number=0;
	}
	else if ( (left_sensor==0) && (center_sensor==0) && (right_sensor==1) )
	{
		state_number=1;
	}
	else if ( (left_sensor==0) && (center_sensor==1) && (right_sensor==0) )
	{
		state_number=2;
	}
	else if ( (left_sensor==0) && (center_sensor==1) && (right_sensor==1) )
	{
		state_number=3;
	}
	else if ( (left_sensor==1) && (center_sensor==0) && (right_sensor==0) )
	{
		state_number=4;
	}
	else if ( (left_sensor==1) && (center_sensor==0) && (right_sensor==1) )
	{
		state_number=5;
	}
	else if ( (left_sensor==1) && (center_sensor==1) && (right_sensor==0) )
	{
		state_number=6;
	}
	else if ( (left_sensor==0) && (center_sensor==1) && (right_sensor==1) )
	{
		state_number=7;
	}
	return state_number;
}

/*Function to Start the training process of the Robot*/
void Start_Robot_Trainig(void){

	uint8 training_LoopCount;	/*Variable to store the counter value in the training function loop*/

	for( training_LoopCount = 0 ; training_LoopCount < training_Count ; training_LoopCount++ )
	{
		uint8 random_action;
		random_action=(uint8)rand();

		Robot_CurrentState=  Get_stateNumber();

			if(random_action==0)
			{

				MoveForward();
				Stop();
				if( (left_sensor==0) && (center_sensor==1) && (right_sensor==0) )
				{
	               /*means that center reads black while left and right read white*/
					Rewards[Robot_CurrentState][random_action]= 5;
					Next_State= Get_stateNumber();
					Update_Q(Next_State, random_action , Rewards[Robot_CurrentState][random_action]);				}
				else
				{
	         		Rewards[Robot_CurrentState][random_action]= -1;
				}
			}
			else if(random_action==1)
			{
				TurnRight();
				Stop();
				if( ( (left_sensor==0) && (center_sensor==0) && (right_sensor==1) ) ||( (left_sensor==0) && (center_sensor==1) && (right_sensor==1) ) )
				{
					/*means that right reads black while left and center reads white or right and center read black so it should turn right*/
					Rewards[Robot_CurrentState][random_action]=5;
					Next_State= Get_stateNumber();
					Update_Q(Next_State, random_action , Rewards[Robot_CurrentState][random_action]);
			    }
			    else
			    {
			    	Rewards[Robot_CurrentState][random_action]= -1;
				}
			}

			else if(random_action==2)
			{
				TurnLeft();
				Stop();
				if( ( (left_sensor==1) && (center_sensor==0) && (right_sensor==0) ) || ( (left_sensor==1) && (center_sensor==1) && (right_sensor==0) ) )
				{
					/*means that left reads black while center and right read white or left and center read black so it should turn left*/
					Rewards[Robot_CurrentState][random_action]= 5;
				    Next_State= Get_stateNumber();
				    Update_Q(Next_State, random_action , Rewards[Robot_CurrentState][random_action]);
				}
				else
				{
					Rewards[Robot_CurrentState][random_action]= -1;
				}
			}
			else if(random_action==3)
			{
				Stop();
				 /*means that all sensors read black*/
				if((left_sensor==1) && (center_sensor==1) && (right_sensor==1))
				{
					Rewards[Robot_CurrentState][random_action]= 5;
					Next_State=Get_stateNumber();
					Update_Q(Next_State, random_action , Rewards[Robot_CurrentState][random_action]);
				}
				else
				{
					Rewards[Robot_CurrentState][random_action]= -1;
				}
			}
	}
}

float Get_max_Q(int stateNum)
{

  float maximum;
  int actionNum;
  maximum=Q[stateNum][0];
  for(actionNum=0; actionNum < 3 ; actionNum++ )
  {
    if(maximum<Q[stateNum][actionNum])
    {
      maximum=Q[stateNum][actionNum];
    }
  }
  return maximum;
}

int Get_max_Q_Index(int stateNum)
{
  int index;
  float max=Q[stateNum][0];
  int actionNum;
  for(actionNum=0;actionNum<3;actionNum++)
  {
    if(max < Q[stateNum][actionNum] )
    {
      max=Q[stateNum][actionNum];
      index=actionNum;
    }
  }
  return index;
}


void Start_Robot_Testing(void)
{
	Robot_CurrentState = Get_stateNumber();
	Robot_Action_ToTake = Get_max_Q_Index(Robot_CurrentState);
	if(Robot_Action_ToTake==0)
	{
		MoveForward();
		_delay_ms(1500);
		Stop();
	}
	else if(Robot_Action_ToTake==1)
	{
		TurnRight();
		_delay_ms(1500);
		Stop();

	}
	else if(Robot_Action_ToTake==2)
	{
		TurnLeft();
		_delay_ms(1500);
		Stop();
	}
	else if(Robot_Action_ToTake==3)
	{
		Stop();
		_delay_ms(1500);
	}
}

int main (void){

	ADC_Init();
	DCmotor_Init();
	while(1){
		ADC_readChannel(0);
		ADC_readChannel(2);
		ADC_readChannel(4);
		Start_Robot_Trainig();
		Start_Robot_Testing();
	}
}
