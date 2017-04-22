// 0.Documentation Section 
// main.c
// Runs on LM4F120 or TM4C123 LaunchPad
// Date: Octobor 26, 2016
//	This program takes Controls a car by bluetooth module 
//	It takes input from bluetooth module then output control signals to control the motors.
//	Car has two motors one for forward and backward movement the other is for left and right steering.
//	Each motor is controlled by two signals (10) or (01) one for each direction.
//	Bluetooth module outputs 6 commands (LEFT, RIGHT, BACKWORD, FORWARD, FORWARD-RIGHT, FORWARD-LEFT, BACKWORD-RIGHT and BACKWORD-LEFT).
//***************************************** External Connections **********************************************************	
//PC6(U3Rx) ==> Bluetooth module Tx 
//PC7(U3Tx) ==> Bluetooth module Rx
//PA2 ==> input to 1st relay (forward) controlling the forward/backword motor
//PA3 ==> input to 2nd relay (backword) controlling the forward/backword motor
//PA6 ==> input to 1st relay (left) controlling the left/right motor
//PA5 ==> input to 2nd relay (right) controlling the left/right motor
//PD0 ==> Light Sensor Forward Left
//PD1 ==> Light Sensor Forward Right
//PD2 ==> Light Sensor Backward Left
//PD3 ==> Light Sensor Backward Right
//PB0 ==> Left Line Sensor
//PB1 ==> Right Line Sensor
//Accelrometer connections
//3.3 v ==> Cs 
//PB3 ==> Sda 
//PB2 ==> Scl 
	// ************************************** 1. Pre-processor Directives Section ********************************************
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "driverlib/adc.h"
#include "inc/hw_memmap.h"
//#include  "inc/hw_ints.h"
#include  "inc/hw_nvic.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "inc/tm4c123gh6pm.h"
//-------------Buffer lengths for bluetooth commands-------------------
#define CMD_BUFFER_LENGTH 40
#define CMD_MAX_LENGTH 10
#define CMD_PARAMETER_MAX_LENGTH 20
#define CMD_MAX_NO_OF_PARAMETERS 5
//--------------To be used as states in Mode_FSM----------------------
#define SIMPLE_CONTROL 0x00
#define FOLLOW_LIGHT 0x01
#define LINE_DETECTOR 0x02
#define MOVE_DISTANCE 0x03
//----Renaming the GPIO pins that are used for controlling the Relays that control the motors--------
#define SYSCTL_PERIPH_OUT_BASE SYSCTL_PERIPH_GPIOA
#define OUT_PORTBASE GPIO_PORTA_BASE
#define OUT_FORWARD GPIO_PIN_2
#define OUT_BACKWARD GPIO_PIN_3
#define OUT_LEFT	GPIO_PIN_6
#define OUT_RIGHT GPIO_PIN_5
#define ACTION_TIME_MS 150  //time to move the motor for each command in SIMPLE CONTROL MODE
//---Renaming the GPIO pins that are connected to the light sensors---------------
#define SYSCTL_PERIPH_PRORXIMITY_SENSOR SYSCTL_PERIPH_GPIOE
#define PRORXIMITY_SENSOR_PORTBASE GPIO_PORTE_BASE
#define PROXIMITY_SENSOR_PIN GPIO_PIN_0

#define SYSCTL_PERIPH_LIGHTSENSORS SYSCTL_PERIPH_GPIOD
#define LIGHTSENSORS_PORTBASE GPIO_PORTD_BASE
#define LIGHTSENSOR_FORWARD_LEFT GPIO_PIN_0
#define LIGHTSENSOR_FORWARD_RIGHT GPIO_PIN_1
#define LIGHTSENSOR_BACKWARD_LEFT	GPIO_PIN_2
#define LIGHTSENSOR_BACKWARD_RIGHT GPIO_PIN_3

//---- each sensor index inside g_sensors array
#define LIGHTSENSOR_FORWARD_LEFT_INDX 0
#define LIGHTSENSOR_FORWARD_RIGHT_INDX 1
#define LIGHTSENSOR_BACKWARD_LEFT_INDX 2
#define LIGHTSENSOR_BACKWARD_RIGHT_INDX 3
#define PROXIMITY_SENSOR_INDX 4

#define LIGHTSENSOR_FORWARD_LEFT_CH ADC_CTL_CH7
#define LIGHTSENSOR_FORWARD_RIGHT_CH ADC_CTL_CH6
#define LIGHTSENSOR_BACKWARD_LEFT_CH ADC_CTL_CH5
#define LIGHTSENSOR_BACKWARD_RIGHT_CH ADC_CTL_CH4
#define PROXIMITY_SENSOR_CH ADC_CTL_CH3
#define COLLISION_DISTANCE 40	//the distance from an obstacle at which the car will stop in cm
#define LIGHT_INTESITY_THRESHOLD 1000
#define READ_LIGHTSENSORS_TIME_MS 250 //the frequency of reading the light sensors output in Millisecons in FollowLight mode
#define ADC_TRIGGER_TIME 200 //the frequency of triggering the ADC in Millisecons
//--------------------BlutoothModule Input-------------------
#define FORWARD_CMD	0xA0
#define BACKWARD_CMD	0xA1
#define LEFT_CMD 0xA2
#define RIGHT_CMD 0xA3
#define FORWARD_LEFT_CMD 0xA4
#define FORWARD_RIGHT_CMD 0xA5
#define BACKWARD_LEFT_CMD 0xA6
#define BACKWARD_RIGHT_CMD 0xA7
#define SIMPLE_CONTROL_MODE 0xB0
#define FOLLOW_LIGHT_MODE 0xB1
#define LINE_DETECTOR_MODE 0xB2
#define MOVE_DISTANCE_MODE 0xB3
// ************************************* 2. Declarations Section *********************************************************
//--------- Global Variables ---------------
unsigned long g_current_mode_state;
uint32_t g_sensors[6] = {0,0,0,0,0,0};  //array to hold the values of sensors(light, proximity, ... etc)
bool g_timer_done;
volatile double X_Axis1=0,Y_Axis1=0,Z_Axis1=0;
volatile int X_Axis2=0 , Y_Axis2=0 , Z_Axis2=0;
volatile double RawX_Axis1=0,RawY_Axis1=0,RawZ_Axis1=0;
volatile long OldV=0,OldA=0,NewA=0,NewV=0;
volatile float Pi=3.14,WheelDiameter=7;
volatile float NumberOfWheelRevs =0,distanceMoved=0,NumberOfCutsBarrier=8,Velocity=0;
volatile float CurrentDistanceMoved=0;
volatile float TotalDistanceMoved=0;
bool FlagRecivedValidNotFinished =0;
volatile bool RecievedNewData;
uint32_t g_velocityRevsCount = 0;
float g_velocity = 0;
uint16_t targetDistance;	
typedef enum {FORWARD, BACKWARD, LEFT, RIGHT, FORWARD_LEFT, FORWARD_RIGHT, BACKWARD_LEFT, BACKWARD_RIGHT, NONE} Direction;

//---------- Function Prototypes --------------
void WriteRegister(int8_t RegisterAddress,int8_t Data);
void ReadAccel(void);
int16_t ReadAccelX(void);
int16_t ReadAccelY(void);
int16_t ReadAccelZ(void);
int32_t stringToInt(char * str);
void sendInt(uint32_t ui32Base, int32_t n, bool positiveSign);
void sendFloat(uint32_t ui32Base, double n, bool positiveSign, uint8_t precession);
void itos(float Number,char *String);
void CalculateDistance(void);//added
void ResetDistanceWheelRevs(void);//added
unsigned int NumbeOfWeelRevsToStop(unsigned int Distance);//added
void PortAInit(void);//added
void Timer2Init(void);//added
void StartTimer2(void);//added
void Timer_Init(void);
void StartTimer(void);
void CalculateVelocity(void);
void EnableI2CModule0(void);
int8_t ReadRegister(int8_t RegisterAddress);
void simpleControl(uint32_t cmd);
void followLight(void);
void lineDetector(void);
void exitCurrentMode(void);
void move(Direction direction);
void moveForward(void);
void moveBackward(void);
void moveLeft(void);
void moveRight(void);
void moveForwardLeft(void);
void moveForwardRight(void);
void moveBackwardLeft(void);
void moveBackwardRight(void);
void SysTickmoveTowardLight_ISR(void);
void setup(void);
bool isValidCtrlCmd(uint32_t cmd);
void UARTSend(uint32_t ui32Base, const char *str);
void ADC_init(void);
void ADC_Timer_Init(uint32_t delay_ms);
void PortB_Init(void);
void Systick_Init(uint32_t delay_ms);
void RelaysPorts_Init(void);
void UART3_Init(void);
void SysTickStopMotors_ISR(void);
void bluetoothHandler(void);
void SystickReadLightSensors_ISR(void);
void ADCConversionFinished_ISR(void);
void stop();
void MoveDistance(Direction dir, uint32_t distance);
void parseCmd(char * bluetoothCmd);
void resolveCmd(char* cmd, char parameters [CMD_MAX_NO_OF_PARAMETERS][CMD_PARAMETER_MAX_LENGTH], char no_of_parameters);
void query(char * cmd);

// --------- Linked data structure ----------
//  Mode Finite State Machine
struct ModeState{
	void (*outPt)(); //output pointer to function
};
typedef const struct ModeState ModeStateType;
ModeStateType mode_FSM[4] = {
	{&simpleControl},	//Simple Control of the car (left-right-forward-backward)
	{&followLight},	//Follow the light which make the car go towards the light using light sensors
	{&lineDetector}, //
	{&MoveDistance}
};


// *********************************** 3. Subroutines Section ***********************************************************
// MAIN: Mandatory for a C Program to be executable
int main (void){
	setup(); //inital configuration of the launchpad
	g_current_mode_state = SIMPLE_CONTROL;
	SysTickIntRegister(SysTickStopMotors_ISR);
	StartTimer2();
	while(1){
		if(g_current_mode_state != SIMPLE_CONTROL && g_current_mode_state != MOVE_DISTANCE){
			mode_FSM[g_current_mode_state].outPt();
			}
		//SysCtlSleep();
	}
}

//---------------------------------------------------isValidCtrlCmd--------------------------------------------------------
//Checks if a given byte is a valid Movement Command [0xA0:0xA7]
//Input: the command you wish to chech wether is valid or not
//Output: True if it's a valid command and False if it is not
bool isValidCtrlCmd(uint32_t cmd){
	return(cmd == FORWARD_CMD
			|| cmd == BACKWARD_CMD
			|| cmd == LEFT_CMD
			|| cmd == RIGHT_CMD
			|| cmd == FORWARD_LEFT_CMD
			|| cmd == FORWARD_RIGHT_CMD
			|| cmd == BACKWARD_LEFT_CMD
			|| cmd == BACKWARD_RIGHT_CMD);
}

//---------------------------------------------------move'Direction'--------------------------------------------------------
//Set the appropriate output on the pins controlling the Relays which controls the motors of the car
//based on the direction
//Input: none
//Output: none
void moveForward(){
		GPIOPinWrite(OUT_PORTBASE, 
		(OUT_FORWARD | OUT_BACKWARD | OUT_LEFT | OUT_RIGHT),
			OUT_FORWARD);
}
void moveBackward(){
	GPIOPinWrite(OUT_PORTBASE, 
		(OUT_FORWARD | OUT_BACKWARD | OUT_LEFT | OUT_RIGHT),
		OUT_BACKWARD);
}
void moveRight(){
	GPIOPinWrite(OUT_PORTBASE, 
		(OUT_FORWARD | OUT_BACKWARD | OUT_LEFT | OUT_RIGHT),
		OUT_RIGHT);
}
void moveLeft(){	
	GPIOPinWrite(OUT_PORTBASE, 
		(OUT_FORWARD | OUT_BACKWARD | OUT_LEFT | OUT_RIGHT),
		OUT_LEFT);
}
void moveForwardRight(){
		GPIOPinWrite(OUT_PORTBASE, 
			(OUT_FORWARD | OUT_BACKWARD | OUT_LEFT | OUT_RIGHT),
			(OUT_FORWARD | OUT_RIGHT));
}
void moveForwardLeft(){
		GPIOPinWrite(OUT_PORTBASE, 
			(OUT_FORWARD | OUT_BACKWARD | OUT_LEFT | OUT_RIGHT),
			(OUT_FORWARD | OUT_LEFT));
}
void moveBackwardRight(){
	GPIOPinWrite(OUT_PORTBASE, 
		(OUT_FORWARD | OUT_BACKWARD | OUT_LEFT | OUT_RIGHT),
		(OUT_BACKWARD| OUT_RIGHT));
}
void moveBackwardLeft(){
	GPIOPinWrite(OUT_PORTBASE, 
		(OUT_FORWARD | OUT_BACKWARD | OUT_LEFT | OUT_RIGHT),
		(OUT_BACKWARD | OUT_LEFT));
}
void stop(){
	GPIOPinWrite(OUT_PORTBASE, 
		(OUT_FORWARD | OUT_BACKWARD | OUT_LEFT | OUT_RIGHT),
	~(OUT_FORWARD | OUT_BACKWARD | OUT_LEFT | OUT_RIGHT));
}

//---------------------------------------------------move--------------------------------------------------------
//move the car based on the input command 
//Input: the directin of the movement command
//Output: none
void move(Direction direction){
	switch(direction){
		case FORWARD:
			moveForward();
		break;		
		case BACKWARD:
			moveBackward();
		break;		
		case LEFT:
			moveLeft();
		break;		
		case RIGHT:
			moveRight();
		break;
		case FORWARD_LEFT:
			moveForwardLeft();
		break;
		case FORWARD_RIGHT:
			moveForwardRight();
		break;
		case BACKWARD_LEFT:
			moveBackwardLeft();
		break;
		case BACKWARD_RIGHT:
			moveBackwardRight();
		break;
	}
}

//---------------------------------------------------simpleControl--------------------------------------------------------
//takes the command wished to execute on the car .. move the car based on that command
//and initialize a timer with the time you wish the car to move on that direction .. when the timer ticks stop the motors
//Input: the directin of the movement command [A0:A7]
//Output: none
void simpleControl(uint32_t cmd){
	//if(g_sensors[4] > COLLISION_DISTANCE){
		move(cmd); //move the car based on the command
		Systick_Init(ACTION_TIME_MS);
//	}else{
		//stop();
	//}
}

//---------------------------------------------------SysTickStopMotors_ISR--------------------------------------------------------
//the interrupt service routine of the SysTick timer in case of Simple Control mode
//stops the movement of the car
//Input: none
//Output: none
void SysTickStopMotors_ISR(void){
	switch(g_current_mode_state){
		case SIMPLE_CONTROL:
			stop();
			SysTickDisable();
			SysTickIntDisable();
		break;
	}
}
//---------------------------------------------------followLight--------------------------------------------------------
//the function is called when entering the Follow Light mode
//initialize the ADC and initializes the Systick timer with 500ms which start the ADC conversion
//Input: none 
//Output: none
void followLight(){
	//TODO :: Implement an exitMode function to shut down SYSTICK timer, ADC,.. etc
	ADCProcessorTrigger(ADC0_BASE,0);
	Systick_Init(READ_LIGHTSENSORS_TIME_MS);	//TODO: don't forget to disable systick interrupts when exiting mode coz systick is always runnig in this mode
	SysTickIntRegister(SysTickmoveTowardLight_ISR);//register an interrupt service routine for SysTick timer
	while (g_current_mode_state == FOLLOW_LIGHT){
		//SysCtlSleep();
	}
}

//---------------------------------------------------SysTickmoveTowardLight_ISR--------------------------------------------------------
//moves the car towards the sensor which has the highst value and if that maximum value exceeds the threshold
//g_sensors is a global array which holds the readings of the light sensors
//Input: none 
//Output: none
void SysTickmoveTowardLight_ISR(){
	char i;
	uint32_t max = g_sensors[LIGHTSENSOR_FORWARD_LEFT_INDX]; 
  Direction	mvmt_direction;
	mvmt_direction = FORWARD_LEFT;
	//get the maximum of the array
	for(i = 1; i < 4; i++){
		if(g_sensors[i] > max){
			max = g_sensors[i];
			mvmt_direction = (i == 1)? FORWARD_RIGHT :
											(i == 2)? BACKWARD_LEFT :
											BACKWARD_RIGHT;	
		}
	}
	if(max > LIGHT_INTESITY_THRESHOLD)
		move(mvmt_direction);
	else
		stop();
}

//--------------------------------------------------- SysTick_Init--------------------------------------------------------
///Subroutine to initialize SysTick timer with specified vlaue ... the SysTick timer will fire every period
//input: the period in milliseconds 
//output: none
void Systick_Init(uint32_t delay_ms){
	uint32_t period;
	SysTickDisable(); //Disable during setup
	period = (SysCtlClockGet()*0.001)*delay_ms - 1;	//load the start value.	no of cycles in 1 ms = 0.001/(Period of one cycle) = CLOCK*0.001
	SysTickPeriodSet(period);
	SysTickIntEnable();
	SysTickEnable();
	NVIC_ST_CURRENT_R = 0; //force current value register to reload
}

char bluetoothCmdBuffer[CMD_BUFFER_LENGTH];
//--------------------------------------------------- bluetoothHandler--------------------------------------------------------
//The interrupt service routine of UART3 which is connected to the BT module
//change the current mdoe of the car and the movement of the car based on the input from BT module
//input:  none
//output: none
void bluetoothHandler(){
	char  c;
	//char static bluetoothCmdBuffer[CMD_BUFFER_LENGTH];
	char static cmdIndx;
	UARTIntClear(UART3_BASE, UART_INT_RX);
	c = UARTCharGet(UART3_BASE);
	if(c == '+' || !(cmdIndx < CMD_BUFFER_LENGTH))
		cmdIndx= 0;
	bluetoothCmdBuffer[cmdIndx] = c;
	cmdIndx++;
	if(c == '\n'){
		parseCmd(bluetoothCmdBuffer);
		cmdIndx = 0;
	}
}


//--------------------------------------------------- parseCmd--------------------------------------------------------
//Parses bluetooth command and acts based on this command
//input:  bluetooth command to be parsed
//output: none
void parseCmd(char* bluetoothCmd){
	char cmd[CMD_MAX_LENGTH];
	char parameter[CMD_MAX_NO_OF_PARAMETERS][CMD_PARAMETER_MAX_LENGTH];
	char cmd_i = 1, paramter_i = 0, i = 0;
	while (cmd_i < CMD_BUFFER_LENGTH) {
		if (bluetoothCmd[cmd_i] == '=') {
			cmd[cmd_i - 1] = '\0';
			//printf("%s %d\n", cmd, strlen(cmd));
			break;
		}
		else if (bluetoothCmd[cmd_i] == '?') {
			cmd[cmd_i - 1] = '\0';
			query(cmd);
			return;
		}else  if (bluetoothCmd[cmd_i] == '\n') {
			cmd[cmd_i - 1] = '\0';
			resolveCmd(cmd, parameter, 0);
			return;
		}
		cmd[cmd_i - 1] = bluetoothCmd[cmd_i];
		cmd_i++;
	}
	cmd_i++;
	while (cmd_i < CMD_BUFFER_LENGTH) {
		if (!(paramter_i < CMD_MAX_NO_OF_PARAMETERS)) {
			//Too many parameters. Command is wrong.
			break;
		}
		if (!(i < CMD_PARAMETER_MAX_LENGTH)) {
			//paramter is too long.
			break;
		}
		if (bluetoothCmd[cmd_i] == ',') {
			parameter[paramter_i][i] = '\0';
			//printf("%s %d\n", parameter[paramter_i], strlen(parameter[paramter_i]));
			i = 0;
			paramter_i++;
			cmd_i++;
			continue;
		}else if (bluetoothCmd[cmd_i] == '\n') {
			parameter[paramter_i][i] = '\0';
			resolveCmd(cmd,parameter,paramter_i+1);
			break;
		}
		parameter[paramter_i][i] = bluetoothCmd[cmd_i];
		i++;
		cmd_i++;
	}
	if (cmd_i >= CMD_BUFFER_LENGTH) {
		//Command buffer exhausted, Command is too long.
	}
}


//--------------------------------------------------- query --------------------------------------------------------
//This function is responsible for responding to query commands. (query commands end with '?')
//input:  One of the query commands (without the '?') ex:- MODE?, LIGHTSEN?,..
//output: none
void query(char * cmd){
	if(strcmp(cmd, "OK") == 0){
		UARTSend(UART3_BASE, "+OK\n");
	}else if(strcmp(cmd, "MODE") == 0){
		switch(g_current_mode_state){
			case SIMPLE_CONTROL:
				UARTSend(UART3_BASE, "+MODE:SIMPLECONTROL\n");
				break;
			case FOLLOW_LIGHT:
				UARTSend(UART3_BASE, "+MODE:FOLLOWLIGHT\n");
				break;
			case LINE_DETECTOR:
				UARTSend(UART3_BASE, "+MODE:FOLLOWLINE\n");
				break;
		}
	}else if(strcmp(cmd, "LIGHTSEN") == 0){
		UARTSend(UART3_BASE, "+LIGHTSEN:");
		sendInt(UART3_BASE, g_sensors[LIGHTSENSOR_FORWARD_LEFT_INDX], false);
		UARTCharPut(UART3_BASE, ',');
		sendInt(UART3_BASE, g_sensors[LIGHTSENSOR_FORWARD_RIGHT_INDX], false);
		UARTCharPut(UART3_BASE, ',');
		sendInt(UART3_BASE, g_sensors[LIGHTSENSOR_BACKWARD_LEFT_INDX], false);
		UARTCharPut(UART3_BASE, ',');
		sendInt(UART3_BASE, g_sensors[LIGHTSENSOR_BACKWARD_RIGHT_INDX], false);
		UARTCharPut(UART3_BASE, '\n');
	}else if(strcmp(cmd, "LINESEN") == 0){
		UARTSend(UART3_BASE, "+LINESEN:");
		UARTCharPut(UART3_BASE, GPIOPinRead((uint32_t)GPIO_PORTB_DATA_BITS_R,(GPIO_PIN_0))? '0' : '1');
		UARTCharPut(UART3_BASE, ',');
		UARTCharPut(UART3_BASE, GPIOPinRead((uint32_t)GPIO_PORTB_DATA_BITS_R,(GPIO_PIN_1))? '0' : '1');
		UARTCharPut(UART3_BASE, '\n');
	}else if(strcmp(cmd, "PROXSEN") == 0){
		UARTSend(UART3_BASE, "+PROXSEN:");
		sendInt(UART3_BASE, g_sensors[PROXIMITY_SENSOR_INDX], false);
		UARTCharPut(UART3_BASE, '\n');
	}else if(strcmp(cmd, "VELOCITY") == 0){
		UARTSend(UART3_BASE, "+VELOCITY:");
		sendFloat(UART3_BASE, g_velocity, false, 2);
		UARTCharPut(UART3_BASE, '\n');
	}else if(strcmp(cmd, "DIST") == 0){
		UARTSend(UART3_BASE, "+DIST:");
		sendFloat(UART3_BASE, TotalDistanceMoved, false, 2);
		UARTCharPut(UART3_BASE, '\n');	
	}else if(strcmp(cmd, "ACC") == 0){
		ReadAccel();
	}else{
		UARTSend(UART3_BASE, "+FAIL:InvalidCommand\n");
	}
}


//--------------------------------------------------- resolveCmd --------------------------------------------------------
//This function is responsible for responding to commands that are not query commands. ex:- +MODE=SIMPLECONTROL, +FW=10..
//input:  cmd is the name of the commmand.
//				parameters parameters of that command.
//				no_of_parameters no of parameters based to the function.
// ex:- for command +FW=100 cmd=FW, parameters = ["FW"], no_of_parameters = 1.
//output: none
void resolveCmd(char* cmd, char parameters[CMD_MAX_NO_OF_PARAMETERS][CMD_PARAMETER_MAX_LENGTH], char no_of_parameters){
	Direction direction = NONE;
	
	if(strcmp(cmd, "MODE") == 0){
		if(strcmp(parameters[0], "SIMPLECONTROL") == 0 && g_current_mode_state != SIMPLE_CONTROL){
				exitCurrentMode();
			 //initalization for simple control mode
			g_current_mode_state = SIMPLE_CONTROL;
			UARTSend(UART3_BASE, "+OK\n");
			SysTickEnable();
			SysTickIntRegister(SysTickStopMotors_ISR);
			return;
		}else if(strcmp(parameters[0], "FOLLOWLIGHT") == 0 && g_current_mode_state != FOLLOW_LIGHT){
			exitCurrentMode();
			//initalization for Follow Light mode
			g_current_mode_state = FOLLOW_LIGHT;
			UARTSend(UART3_BASE, "+OK\n");
			return;
		}else if(strcmp(parameters[0], "FOLLOWLINE") == 0 && g_current_mode_state != LINE_DETECTOR){
			exitCurrentMode();
			//initalization for line detector mode
			g_current_mode_state = LINE_DETECTOR;
			UARTSend(UART3_BASE, "+OK\n");
			return;
		}
	}else if(strcmp(cmd, "RSTDIST") == 0){
		TotalDistanceMoved = 0;
		UARTSend(UART3_BASE, "+OK\n");
		return;
	}
	
	if(g_current_mode_state == SIMPLE_CONTROL){
		if(strcmp(cmd, "FW") == 0){
			direction = FORWARD;
		}else if(strcmp(cmd, "BW") == 0){
			direction = BACKWARD;
		}else if(strcmp(cmd, "LT") == 0){
			direction = LEFT;
		}else if(strcmp(cmd, "RT") == 0){
			direction = RIGHT;
		}else if(strcmp(cmd, "FL") == 0){
			direction = FORWARD_LEFT;
		}else if(strcmp(cmd, "FR")== 0){
			direction = FORWARD_RIGHT;
		}else if(strcmp(cmd, "BL")== 0){
			direction = BACKWARD_LEFT;
		}else if(strcmp(cmd, "BR")== 0){
			direction = BACKWARD_RIGHT;
		}
		if(direction != NONE){
			if(no_of_parameters == 0){ //simple control Command without distance
				move(direction);
				Systick_Init(ACTION_TIME_MS);	
				return;
			}else if(no_of_parameters == 1){ //simple control command with distance
				int distance = stringToInt(parameters[0]);
				if(distance > 0)
					MoveDistance(direction, stringToInt(parameters[0]));
				else
					UARTSend(UART3_BASE, "+FAIL:InvalidValueForDistance\n");
				return;
			}
		}
		UARTSend(UART3_BASE, "+FAIL:InvalidCommand\n");
	}
}


//--------------------------------------------------- RelaysPorts_Init--------------------------------------------------------
///The initialization of the ports controlling the relays that control the motors of the car
//input:  none
//output: none
void RelaysPorts_Init(){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_OUT_BASE);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_OUT_BASE));
	GPIOPinTypeGPIOOutput(OUT_PORTBASE,
		OUT_FORWARD | OUT_BACKWARD | OUT_LEFT | OUT_RIGHT);
	stop(); //initialize the movment of the car by stopping all the motors
}
		
//--------------------------------------------------- UART3_Init--------------------------------------------------------
//initialization of UART3 PC[6:7] which is connected to the BTmodule
//input:  none
//output: none
void UART3_Init(){
	//initalize portc
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));
  GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	GPIOPinConfigure(GPIO_PC7_U3TX);
	GPIOPinConfigure(GPIO_PC6_U3RX);
	//configure uart
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART3));
	UARTEnable(UART3_BASE);
	UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 9600 , //9600 baud rate, 8N1
		(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |			
		UART_CONFIG_PAR_NONE));
	//enable FIFO mode for UART 3
	UARTFIFODisable(UART3_BASE);
	UARTIntRegister(UART3_BASE, bluetoothHandler);
	//FIFO levels defines the amount of bytes before generating the interrupt 
	//RX1 -> 2 bytes , RX2->4 bytes
	IntEnable(INT_UART3);
	UARTIntEnable(UART3_BASE, UART_INT_RX );
}

//---------------------------------------------------  ADC_init--------------------------------------------------------
//initialization of ADC
//Configure ADC0 -- Sequencer 0
//input:  none
//output: none
void ADC_init(){
	//Configure ADC0 -- Sequence number 0-1-2-3 -- 
	SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
	ADCSequenceConfigure(ADC0_BASE,0, ADC_TRIGGER_PROCESSOR,1);
	ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_FULL, 1);
  ADCSequenceStepConfigure(ADC0_BASE,0,LIGHTSENSOR_FORWARD_LEFT_INDX, LIGHTSENSOR_FORWARD_LEFT_CH);
	ADCSequenceStepConfigure(ADC0_BASE,0,LIGHTSENSOR_FORWARD_RIGHT_INDX, LIGHTSENSOR_FORWARD_RIGHT_CH);
  ADCSequenceStepConfigure(ADC0_BASE,0,LIGHTSENSOR_BACKWARD_LEFT_INDX, LIGHTSENSOR_BACKWARD_LEFT_CH);
  ADCSequenceStepConfigure(ADC0_BASE,0,LIGHTSENSOR_BACKWARD_RIGHT_INDX, LIGHTSENSOR_BACKWARD_RIGHT_CH);
  ADCSequenceStepConfigure(ADC0_BASE,0,PROXIMITY_SENSOR_INDX, PROXIMITY_SENSOR_CH | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 0);
	// Enable conversion complete interrupt for the ADC
	ADCIntEnable(ADC0_BASE, 0);
	ADCIntRegister(ADC0_BASE, 0, ADCConversionFinished_ISR);
	// Finally Enable the sequence
	ADCSequenceEnable(ADC0_BASE, 0);
	ADC0_CC_R = 0x01; //ADC clock source to be PIOSC (workaround Errata ADC #8)
	ADC_Timer_Init(ADC_TRIGGER_TIME);
}


void PortB_Init(){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);   // Enable the GPIOB peripheral
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)); 	// Wait for the GPIOB module to be ready.
	// Initialize the GPIO pin configuration.
	GPIOPinTypeGPIOInput((uint32_t)GPIO_PORTB_DATA_BITS_R, (GPIO_PIN_0 | GPIO_PIN_1));
}

//---------------------------------------------------  lightSensorsInit--------------------------------------------------------
//Initialization of light sensors ports 
//input:  none
//output: none
void lightSensorsInit(){
		SysCtlPeripheralEnable(SYSCTL_PERIPH_LIGHTSENSORS);
		while(!SysCtlPeripheralReady(SYSCTL_PERIPH_LIGHTSENSORS));
		GPIOPinTypeADC(LIGHTSENSORS_PORTBASE, LIGHTSENSOR_FORWARD_LEFT | 
		LIGHTSENSOR_FORWARD_RIGHT|
		LIGHTSENSOR_BACKWARD_LEFT|
		LIGHTSENSOR_BACKWARD_RIGHT);
}

//---------------------------------------------------  proximitySensorInit--------------------------------------------------------
//Initialization of proximity sensor port 
//input:  none
//output: none
void proximitySensorInit(){
		SysCtlPeripheralEnable(SYSCTL_PERIPH_PRORXIMITY_SENSOR);
		while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PRORXIMITY_SENSOR));
		GPIOPinTypeADC(PRORXIMITY_SENSOR_PORTBASE, PROXIMITY_SENSOR_PIN);
}

//---------------------------------------------------calculateObstacleDistance--------------------------------------------------------
//Convert the reading of proximity sensor to dsitance in cm to the obstacle
//Input: proximity sensor reading 
//Output: none
uint32_t calculateObstacleDistance(uint32_t proximitySensorReading){
	return 39312000/(proximitySensorReading*1551);
}

//---------------------------------------------------ADCConversionFinished_ISR--------------------------------------------------------
//this is the interrupt service routine of the ADC for Follow Light mode
//take the readings of the light sensors and store it in the golobal array of g_sensors
//Input: none 
//Output: none
void ADCConversionFinished_ISR(void){
		char num;
		char i;
		char str[30];
		ADC0_ISC_R |= 0x01; //write one to clear the interrupt
		num = ADCSequenceDataGet(ADC0_BASE, 0, g_sensors);
		g_sensors[4] = calculateObstacleDistance(g_sensors[4]);
	
}


//---------------------------------------------------  ADC_Timer_Handler --------------------------------------------------------
//Handler of ADC_Timer, triggers the ADC each time the handler reaches zero 
//input:  none
//output: none
void ADC_Timer_Handler(){
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	ADCProcessorTrigger(ADC0_BASE,0); 
	g_velocity = ((g_velocityRevsCount * (Pi * NumberOfCutsBarrier)/ WheelDiameter) / 0.2) /100;
	g_velocityRevsCount= 0;
	//ReadAccel();
}

//---------------------------------------------------  ADC_Timer_Init--------------------------------------------------------
//Initialization of periodic Timer0 that controls when ADC triggers 
//input:  none
//output: none
void ADC_Timer_Init(uint32_t delay_ms){
	uint32_t period = (SysCtlClockGet()*0.001)*delay_ms - 1;	//load the start value.	no of cycles in 1 ms = 0.001/(Period of one cycle) = CLOCK*0.001
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER0_BASE, TIMER_A, period);
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER0_BASE, TIMER_A);
	TimerIntRegister(TIMER0_BASE, TIMER_A, ADC_Timer_Handler);
}

//---------------------------------------------------setup--------------------------------------------------------
//initialization of the Ports connected to the relays
//initialization of the UART3 and sets the clock of the launchpad to be 16Mhz and initializes the SleepMode
//Configure ADC0 -- Sequencer 0 -- Channels CH0 CH1 CH2 CH3 (PE[0:3])
//input:  none
//output: none
void setup(){
	SysCtlClockSet(SYSCTL_SYSDIV_12_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |SYSCTL_XTAL_16MHZ);  //SetClock as 16Mhz
	UART3_Init();
	EnableI2CModule0(); // Enable module I2C0
	Timer2Init();//added
	WriteRegister(0x31,0x0B); // setting resolution 
	WriteRegister(0x2D,0x08); // disable sleep mode	 
	PortAInit();//reading acceleration with interrupts driven 
	PortB_Init();
	IntMasterEnable();
	ADC_init();
	RelaysPorts_Init();
	lightSensorsInit();
	SysCtlPeripheralClockGating(true);
	//SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART1);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART3);
	SysCtlSleepPowerSet(SYSCTL_FLASH_LOW_POWER|SYSCTL_SRAM_LOW_POWER);
	SysCtlLDOSleepSet(SYSCTL_LDO_0_90V);
}


//---------------------------------------------------UARTSend--------------------------------------------------------
//sends multiple chars via UART
//input:  the UART you wish to use for sending , and a pointer to the char array you wish to send
//output: none
void UARTSend(uint32_t ui32Base, const char *str){
	int32_t count = strlen(str);
    // Loop while there are more characters to send.
    while(count--){
        // Write the next character to the UART.
        UARTCharPut(ui32Base, *str++);
    }
}

//---------------------------------------------------sendFloat--------------------------------------------------------
//sends a floating point number through UART as string
//input: ui32Base 		The base of the UART you wish to use for sending
//			 n 						floating point number to be sent.
//			 positiveSign true to display a '+' sign for positive numbers	
//			 precession		the number of digits after the decimal point to send. Maximum is 9, if the function is called with more than 9, only 9 digits after decimal point are sent
//output: none

void sendFloat(uint32_t ui32Base, double n, bool positiveSign, uint8_t precession){
  int32_t integralPart;
	uint32_t multiplier, maxMultiplier;
	float	fractionalPart;
	//send sign
  if (n < 0)
		UARTCharPut(ui32Base, '-');
	else if(positiveSign)
		UARTCharPut(ui32Base, '+');
	
	//split number to integral and fractional part
	n = fabs(n);
	integralPart = (int32_t) n;
	fractionalPart = n - integralPart;
	
	//send integral part
	sendInt(ui32Base, integralPart, false);
	
	if(precession == 0)
		return;
	//send decimal point
	UARTCharPut(ui32Base, '.');
	
	//send fractional part
	maxMultiplier = precession == 0 ? 1:
									precession == 1 ? 10:
									precession == 2 ? 100:
									precession == 3 ? 1000:
									precession == 4 ? 10000:
									precession == 5 ? 100000:
									precession == 6 ? 1000000:
									precession == 7 ? 10000000:
									precession == 8 ? 100000000: 1000000000;
	
	for (multiplier = 10; multiplier <= maxMultiplier; multiplier *= 10) {
		int digitAfterDecimal = (int32_t)(fractionalPart * multiplier) % 10;
		UARTCharPut(UART3_BASE, '0' + digitAfterDecimal);
	}
}

//---------------------------------------------------sendInt--------------------------------------------------------
//sends an integer number through UART as string
//input: ui32Base the UART you wish to use for sending
//			 n integer number to be sent.
//			positiveSign true to display a '+' sign for positive numbers
//output: none

void sendInt(uint32_t ui32Base, int32_t n, bool positiveSign){
	char j;
  uint32_t divider = (n < 10 ? 1 :   
      (n < 100 ? 10 :   
      (n < 1000 ? 100 :   
      (n < 10000 ? 1000 :   
      (n < 100000 ? 10000 :   
      (n < 1000000 ? 100000 :   
      (n < 10000000 ? 1000000 :  
      (n < 100000000 ? 10000000 :  
			(n < 1000000000 ? 100000000 :  
        1000000000)))))))));
	char noOfDigits = (n < 10 ? 1 :
		(n < 100 ? 2 :
		(n < 1000 ? 3 :
		(n < 10000 ? 4 :
		(n < 100000 ? 5:
		(n < 1000000 ? 6:
		(n < 10000000 ? 7:
		(n < 100000000 ? 8:
		(n < 1000000000 ? 9:
		10)))))))));
	
  if (n < 0)
		UARTCharPut(ui32Base, '-');
	else if(positiveSign)
		UARTCharPut(ui32Base, '+');

	for(j = 0; j < noOfDigits; j++){
		char c = (char)'0' + n / divider;
		UARTCharPut(ui32Base, c);
		n = n%divider;
		divider /= 10;
	}
}
//---------------------------------------------------stringToInt--------------------------------------------------------
//convert numbers in string into integers
//input: ui32Base the UART you wish to use for sending
//			 n integer number to be sent.
//output: none

int32_t stringToInt(char * str) {
	char i = 0 ;
	int32_t result = 0, multiplier;
	char length = strlen(str);
	bool negative = false;
	if (str[0] == '-') {
		negative = true;
		i = 1;
		length--;
	}
	multiplier = (length == 1 ? 1 :
		(length == 2 ? 10 :
			(length == 3 ? 100 : 
				(length == 4 ? 1000 :
					(length == 5 ? 10000 :
						(length == 6 ? 100000 :
							(length == 7 ? 1000000 :
								(length == 8 ? 10000000 :
									(length == 9 ? 100000000 :
										1000000000)))))))));;
	if (negative)	length++;
	for (;i < length; i++, multiplier /= 10) {
		result += (str[i]-'0') * multiplier;
	}
	return  negative? (0-result): result;
}

//---------------------------------------------------exitCurrentMode--------------------------------------------------------
//exits the mode that currently running on the car
//input:  none
//output: none
void exitCurrentMode(){
	switch(g_current_mode_state){
		case SIMPLE_CONTROL:
			stop();
			SysTickIntUnregister();
			SysTickDisable();
			SysTickIntDisable();
			break;
		case FOLLOW_LIGHT:
			stop();
			SysTickIntUnregister();
			SysTickDisable();
			SysTickIntDisable();
			break;
		case LINE_DETECTOR:
			stop();
			SysTickIntUnregister();
			SysTickDisable();
			SysTickIntDisable();
			break;
		case MOVE_DISTANCE:
			stop();
			ResetDistanceWheelRevs();
			//UARTFIFODisable(UART3_BASE);
			break;
	}
}

//---------------------------------------------------SystickLineDetector_ISR--------------------------------------------------------
//exits the mode that currently running on the car
//input:  none
//output: none
void SystickLineDetector_ISR(){
	g_timer_done = true;
}

void lineDetector(){
	 
	int left_sensor , right_sensor ,count=21;
	char state = 'c', command;
	bool checkright = true , stop = false;
	int count_loop = 25;

	int run_delay = 35;
	int stop_delay = 35;
	g_timer_done = true;
	SysTickIntRegister(SystickLineDetector_ISR);
	
	while (g_current_mode_state == LINE_DETECTOR){
	
	while (!g_timer_done && g_current_mode_state == LINE_DETECTOR){};   //wait for run
  g_timer_done = false ;		
		
	GPIOPinWrite(OUT_PORTBASE, (OUT_FORWARD | OUT_BACKWARD),~(OUT_FORWARD | OUT_BACKWARD));	//stop
	Systick_Init(stop_delay);
		
	while (!g_timer_done && g_current_mode_state == LINE_DETECTOR){}; //wait for stop
	g_timer_done = false ;

	if(stop){
			state = 'w';
			command = 'w';
		}
		
	left_sensor = GPIOPinRead((uint32_t)GPIO_PORTB_DATA_BITS_R,(GPIO_PIN_0));
	right_sensor = GPIOPinRead((uint32_t)GPIO_PORTB_DATA_BITS_R,(GPIO_PIN_1));
		
	if(left_sensor == 0 && right_sensor == 0){
	    	count = 0 ;
				stop = false;
				if(state == 'b'){
					if (checkright){
						state = 'y';
						command = 'r';
					}else{
						state = 's';
						command = 'l';
					}
				}else{
						state = 'c';
						command = 'f';
				}
	 }else if(left_sensor == 0 && right_sensor == 0x02){
		 
					count = 0 ;
					stop = false;
		 
		 if(state == 'r'){
					state = 'c';
					command = 'f';
		 }else{
			   	state = 'l';
					command = 'l';
		 }
		 
	 }else if(left_sensor == 1 && right_sensor == 0){
		 
					count = 0 ;
					stop = false;
		 
		 if(state == 'l'){
					state = 'c';
					command = 'f';
		 }else{
					state = 'r';
					command = 'r';
		 }
		 
	 }else{  // left and right = 1
		 
			 if (count > count_loop){
					state = 'w';
					command = 'w';
			 }else{
				 if(state == 'c'){
						state = 'b' ;
						command = 'b';
				 }else if (state == 'r'){
						state = 'r';
						command = 'r';
						count += 1;
				 }else if (state == 'l'){
						state = 'l';
						command = 'l';
						count += 1;
				 }else if (state == 'b'){
						state = 'b';
						command = 'b';
					  count += 2;
				 }else if (state == 'y'){
						state = 'b';
						checkright = false;
						command = 'y'; //back right
				 }else if (state == 's'){
						state = 'b';
						checkright = true ; 
						stop = true ;
						command = 's' ; //back left
				 }else{
						state = 'w';
						command = 'w';
				 }
			 }
	 }	
	 
	 if (g_sensors[4] < 15){
		 state = 'b';
		 command = 'b';
	 }

	 switch(command){
		 case 'f': GPIOPinWrite(OUT_PORTBASE,(OUT_FORWARD | OUT_BACKWARD | OUT_LEFT | OUT_RIGHT), OUT_FORWARD);
			break;
		 case 'r': GPIOPinWrite(OUT_PORTBASE,(OUT_FORWARD | OUT_BACKWARD | OUT_LEFT | OUT_RIGHT),(OUT_FORWARD | OUT_RIGHT));
			break;
		 case 'l': GPIOPinWrite(OUT_PORTBASE,(OUT_FORWARD | OUT_BACKWARD | OUT_LEFT | OUT_RIGHT),(OUT_FORWARD | OUT_LEFT));
			break;
		 case 'b': GPIOPinWrite(OUT_PORTBASE,(OUT_FORWARD | OUT_BACKWARD | OUT_LEFT | OUT_RIGHT),OUT_BACKWARD);
			break;
		 case 'y': GPIOPinWrite(OUT_PORTBASE,(OUT_FORWARD | OUT_BACKWARD | OUT_LEFT | OUT_RIGHT),(OUT_BACKWARD| OUT_RIGHT));
			break;
		 case 's': GPIOPinWrite(OUT_PORTBASE,(OUT_FORWARD | OUT_BACKWARD | OUT_LEFT | OUT_RIGHT),(OUT_BACKWARD | OUT_LEFT));
			break;
		 case 'w': GPIOPinWrite(OUT_PORTBASE,(OUT_FORWARD | OUT_BACKWARD | OUT_LEFT | OUT_RIGHT),~(OUT_BACKWARD | OUT_LEFT | OUT_FORWARD | OUT_RIGHT));
	 }
	 
	 Systick_Init(run_delay);
	 //```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````
	
	//GPIOPinWrite(OUT_PORTBASE, (OUT_FORWARD | OUT_BACKWARD), ~(OUT_FORWARD | OUT_BACKWARD));
 }	
}

//Essam's starts here
//convert floats to string provided that the number is 1 digit before decimal point
//can be enhanced to include more than 1 digit before the decimal point
void itos(float Number,char *StringToConvert)
{
	int IntOfNumber=Number*100;
	if(Number<0)
	{
		StringToConvert[1]='-';
		IntOfNumber=IntOfNumber*-1;
	}
	else
	{
		StringToConvert[1]='+';
	}
		
	StringToConvert[2]=(IntOfNumber/100)%10 + 0x30;
	StringToConvert[3]='.';
	StringToConvert[4]=(IntOfNumber/10)%10 + 0x30;
	StringToConvert[5]= IntOfNumber%10 + 0x30;
}

void ReadAccel(void){
		char X_Accel[7],Y_Accel[7],Z_Accel[7],acceleration[20];
		X_Accel[0]='x';
		Y_Accel[0]='y';
		Z_Accel[0]='z';
		RawX_Axis1=ReadAccelX();
		X_Axis1 = (RawX_Axis1 * 0.00390625+0.35);
		//X_Axis2 = X_Axis1*100;
		RawY_Axis1=ReadAccelY();
		Y_Axis1 = RawY_Axis1 * 0.00390625+0.08;
		//Y_Axis2 = Y_Axis1*100;
		RawZ_Axis1=ReadAccelZ();
		Z_Axis1 = (RawY_Axis1 *  0.00390625)+1.08;
		//Z_Axis1 = Z_Axis1*100;
		//Z_Axis2 = Z_Axis1;
		itos(X_Axis1,X_Accel);
		itos(Y_Axis1,Y_Accel);
		itos(Z_Axis1,Z_Accel);
		/*old code for acceleration before function creation
	acceleration[0]='x';
		acceleration[1]='+';
		if((X_Axis2)<0)
		{
			acceleration[1]='-';
			X_Axis2=X_Axis2*-1;
		}
		acceleration[2]=(X_Axis2/100)%10 + 0x30;
		acceleration[3]='.';
		acceleration[4]=(X_Axis2/10)%10 + 0x30;
		acceleration[5]= (X_Axis2%10) + 0x30;
		acceleration[6]=' ';
		acceleration[7]='Y';
		acceleration[8]='+';
		if((Y_Axis2)<0)
		{
			acceleration[8]='-';
			Y_Axis2=Y_Axis2*-1;
		}
		acceleration[9]=(Y_Axis2/100)%10 + 0x30;
		acceleration[10]='.';
		acceleration[11]=(Y_Axis2/10)%10 + 0x30;
		acceleration[12]= Y_Axis2%10 + 0x30;
		acceleration[13]=' ';
		acceleration[14]='Z';
		acceleration[15]='+';
		if((Z_Axis2)<0)
		{
			acceleration[15]='-';
			Z_Axis2=Z_Axis2*-1;
		}
		acceleration[16]=(Z_Axis2/100)%10 + 0x30;
		acceleration[17]='.';
		acceleration[18]=(Z_Axis2/10)%10 + 0x30;
		acceleration[19]= Z_Axis2%10 + 0x30;
		//sprintf(acceleration,"x%.2f y%.2f z%.2f",X_Axis1,Y_Axis1,Z_Axis1);
		*/
		sendFloat(UART3_BASE, X_Axis1, true, 2);
		//UARTSend(UART3_BASE,X_Accel);
		UARTCharPut(UART3_BASE,',');
		//UARTSend(UART3_BASE,Y_Accel);
		sendFloat(UART3_BASE, Y_Axis1, true, 2);
		UARTCharPut(UART3_BASE,',');		
		//UARTSend(UART3_BASE,Z_Accel);
		sendFloat(UART3_BASE, Z_Axis1, true, 2);
		UARTCharPut(UART3_BASE,'\n');	
		
}

void EnableI2CModule0(void)
{
	volatile int Delay=0;
	SYSCTL_RCGCI2C_R|=0x00000001; //set i2c module 0 clock active
	Delay=SYSCTL_RCGCI2C_R; //delay allow clock to stabilize 
	SYSCTL_RCGCGPIO_R |=0x00000002; //i2c module 0 is portB so activate clock for port B
	Delay = SYSCTL_RCGCGPIO_R; //delay allow clock to stabilize 
	GPIO_PORTB_AFSEL_R|= 0x0000000C; //enable alternate functions for PB2 and PB3
	GPIO_PORTB_ODR_R |= 0x00000008; //set PB3 (I2C SDA)  for open drain
	GPIO_PORTB_DEN_R |= 0xFF; //Enable digital on Port B
	GPIO_PORTB_PCTL_R |=0x03;
	I2C0_PP_R |= 0x01;
	I2C0_MTPR_R = 0x0000007; //set SCL clock ( change this if u plan to run on 16 MHZ ) 
	I2C0_MCR_R |= 0x00000010; //intialize mcr rigester with that value given in datasheet
}
int8_t ReadRegister(int8_t RegisterAddress)
{
	volatile int32_t mcs = I2C0_MCS_R ;
	volatile int8_t result=0;
	I2C0_MSA_R = 0x000000A6; //write operation
	I2C0_MDR_R = RegisterAddress; //place data to send mdr register
	I2C0_MCS_R = 0x00000007; //stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	I2C0_MSA_R = 0x000000A7; // read operation
	I2C0_MCS_R = 0x00000007; // stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	result = I2C0_MDR_R;
	return result;
}

void WriteRegister(int8_t RegisterAddress,int8_t Data)
{
	volatile int32_t mcs = I2C0_MCS_R ;
	I2C0_MSA_R = 0x000000A6; //write operation
	I2C0_MDR_R = RegisterAddress; //place register address to set in mdr register
	I2C0_MCS_R = 0x00000003; //burst send ( multiple bytes send ) 
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	I2C0_MDR_R = Data; //place data to be sent in  mdr register
	I2C0_MCS_R = 0x00000005; // transmit followed by stop state 
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
}
int16_t ReadAccelX(void)
{
	volatile int16_t result=0,LSB=0,MSB=0;
	I2C0_MSA_R = 0x000000A6; //write operation
	I2C0_MDR_R = 0x32; //data x0
	I2C0_MCS_R = 0x00000007; //stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	I2C0_MSA_R = 0x000000A7; // read operation
	I2C0_MCS_R = 0x00000007; // stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	LSB = I2C0_MDR_R;
	//result above is LSB of accel x
	//repeat to get MSB
	I2C0_MSA_R = 0x000000A6; //write operation
	I2C0_MDR_R = 0x33; //data x1
	I2C0_MCS_R = 0x00000007; //stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	I2C0_MSA_R = 0x000000A7; // read operation
	I2C0_MCS_R = 0x00000007; // stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit
	MSB=I2C0_MDR_R;
	MSB= MSB <<8;
	result = result | LSB;
	result = result | MSB;
return result;	
}
int16_t ReadAccelY(void)
{
	volatile int16_t result=0,LSB=0,MSB=0;
	I2C0_MSA_R = 0x000000A6; //write operation
	I2C0_MDR_R = 0x34; //data Y0
	I2C0_MCS_R = 0x00000007; //stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	I2C0_MSA_R = 0x000000A7; // read operation
	I2C0_MCS_R = 0x00000007; // stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	LSB = I2C0_MDR_R;
	//result above is LSB of accel Y
	//repeat to get MSB
	I2C0_MSA_R = 0x000000A6; //write operation
	I2C0_MDR_R = 0x35; //data Y1
	I2C0_MCS_R = 0x00000007; //stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	I2C0_MSA_R = 0x000000A7; // read operation
	I2C0_MCS_R = 0x00000007; // stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit
	MSB=I2C0_MDR_R;
	MSB= MSB <<8;
	result = result | LSB;
	result = result | MSB;
return result;	
}
int16_t ReadAccelZ(void)
{
	volatile int16_t result=0,LSB=0,MSB=0;
	I2C0_MSA_R = 0x000000A6; //write operation
	I2C0_MDR_R = 0x36; //data Z0
	I2C0_MCS_R = 0x00000007; //stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	I2C0_MSA_R = 0x000000A7; // read operation
	I2C0_MCS_R = 0x00000007; // stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	LSB = I2C0_MDR_R;
	//result above is LSB of accel Z
	//repeat to get MSB
	I2C0_MSA_R = 0x000000A6; //write operation
	I2C0_MDR_R = 0x37; //data xZ1
	I2C0_MCS_R = 0x00000007; //stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	I2C0_MSA_R = 0x000000A7; // read operation
	I2C0_MCS_R = 0x00000007; // stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit
	MSB=I2C0_MDR_R;
	MSB= MSB <<8;
	result = result | LSB;
	result = result | MSB;
return result;	
}
void Timer_Init(void)
{
	SYSCTL_RCGCTIMER_R |= 0x00000002; // enable timer module clock
	TIMER1_CTL_R &=0xFFFFFFFE; // clear enable bit 
	TIMER1_CFG_R &=0x8; // setting to one shot timer mode
	TIMER1_TAMR_R  |= 0x00000001; // setting to one shot timer mode
	TIMER1_TAILR_R = 0x04C4B400; //preload the value with this for 1 second , u can preload with 0xF42400 when using 16 MHZ clock	
}
void StartTimer(void)
{
	TIMER1_CTL_R |=0x0F; // start timer ( set enable bit ) needs fixing should be 01 will look at it later
}
void CalculateVelocity(void){
	char ToSend[10];
	//velocity calculated in cm/sec
	Velocity=(TotalDistanceMoved-CurrentDistanceMoved)*16000000/0x00F42400;
	//convert to m/s
	Velocity = Velocity/100;
	//itos(Velocity,ToSend);
	//UARTSend(UART3_BASE,"Velocity: ");
	//UARTSend(UART3_BASE,ToSend);
	//UARTCharPut(UART3_BASE,'\n');
}

void PortAInit(void){
  volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000001; // activate clock for port A
  delay = SYSCTL_RCGC2_R; 			// delay allow clock to stabilize 
  GPIO_PORTA_DIR_R &= ~0x80;    // make PA7 input ( this will be the reciever diode)
  GPIO_PORTA_AFSEL_R &= ~0x80;  // disable alt funct on PA7
  GPIO_PORTA_DEN_R |= 0x80;     // enable digital I/O on PA7   
  GPIO_PORTA_PCTL_R &= ~0xF0000000; // configure PF4 as GPIO
  GPIO_PORTA_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTA_PDR_R |= 0x80;     //     enable weak pull-Down on PA7
  GPIO_PORTA_IS_R &= ~0x80;     // PA7 is edge-sensitive
  GPIO_PORTA_IBE_R &= ~0x80;    //     PA7 is not both edges
  GPIO_PORTA_IEV_R |= 0x80;    //     PA7 falling edge event
  GPIO_PORTA_ICR_R = 0x80;      // clear flag7
  GPIO_PORTA_IM_R |= 0x80;      // arm interrupt on PA7
  NVIC_PRI0_R = (NVIC_PRI0_R&0xFFFFFF0F)|0x00000010; // (g) priority 1
  NVIC_EN0_R = 0x00000001;      // enable interrupt 0 in NVIC
  
}
void Timer2Init(void){
	SYSCTL_RCGCTIMER_R |= 0x00000004; // enable timer module clock
	TIMER2_CTL_R &= 0xFFFFFFFE; //disable timer
	TIMER2_CFG_R &= 0x00000000;
	TIMER2_TAMR_R |= 0x00000002; // periodic mode
	TIMER2_TAILR_R =0x00F42400; //8 million ticks for 1/2 a sec delay
	TIMER2_IMR_R |=0x00000001; //enable interrupts for timer A
	NVIC_PRI5_R |= (NVIC_PRI5_R&0x0FFFFFFF)|0xA0000000; // (g) priority 5
	NVIC_EN0_R |= 0x00800000;      // enable interrupt 0 in NVIC
}

void StartTimer2(void){
	TIMER2_CTL_R |= 0x00000001; //enable timer
}

void TIMER2A_Handler(void){
		//ReadAccel();
		CalculateVelocity();
		CurrentDistanceMoved=TotalDistanceMoved;
		TIMER2_RIS_R = 0xFFFFFFFF;
		TIMER2_ICR_R = 0xFFFFFFFF;	
}

//Interrupt handler add the count of quarter revs by 1 and start or stop timer based
//on current count even / odd to measure velocity later on
void GPIOA_Handler(void){
	NumberOfWheelRevs = NumberOfWheelRevs + 1;
	g_velocityRevsCount++;
	TotalDistanceMoved += (  Pi / NumberOfCutsBarrier ) * WheelDiameter;
	CalculateDistance();
	if(distanceMoved >= targetDistance&& targetDistance != 0){
		ResetDistanceWheelRevs();	
		stop();
		UARTSend(UART3_BASE, "+OK\n");
	}
	GPIO_PORTA_ICR_R = 0x80;
	//check the count number if even and not equal to 0 calculate speed then reset timer
	//otherwise if the count is odd then just enable the counter to start counting
}

//calculate total distance covered using number of cuts in barrier , wheel revs done
//and wheel diameter
void CalculateDistance(void){
	distanceMoved = NumberOfWheelRevs * (  Pi / NumberOfCutsBarrier ) * WheelDiameter;
}	

//reset the total distance covered 
void ResetDistanceWheelRevs(void){
	distanceMoved =0;
	NumberOfWheelRevs =0;
	targetDistance = 0;
}

unsigned int NumbeOfWeelRevsToStop(unsigned int distance){
	return (distance/(Pi*WheelDiameter))*4;
}

//to be changed input is first byte recieved
void MoveDistance(Direction dir ,uint32_t distance){
	ResetDistanceWheelRevs();
	if( distance == 0)
		return;
	targetDistance = distance > 10? distance - 10 : distance; //Error 10 CM
	move(dir);	
}

