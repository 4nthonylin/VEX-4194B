#pragma config(Motor,  port1,           LeftDrive1,    tmotorVex393, openLoop, reversed)
#pragma config(Motor,  port2,           LeftDrive2,    tmotorVex393, openLoop, reversed)
#pragma config(Motor,  port3,           LeftLift1,     tmotorVex393, openLoop)
#pragma config(Motor,  port4,           LeftLift2,     tmotorVex393, openLoop)
#pragma config(Motor,  port5,           LeftRoller,    tmotorVex393, openLoop)
#pragma config(Motor,  port6,           RightRoller,   tmotorVex393, openLoop, reversed)
#pragma config(Motor,  port7,           RightLift2,    tmotorVex393, openLoop, reversed)
#pragma config(Motor,  port8,           RightLift1,    tmotorVex393, openLoop, reversed)
#pragma config(Motor,  port9,           RightDrive2,   tmotorVex393, openLoop)
#pragma config(Motor,  port10,          RightDrive1,   tmotorVex393, openLoop)
#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(105)

#include "Vex_Competition_Includes.c"

int lift_rates[] = {127, -10, 10};
int lift_state = 0; //0 up, 1 down, 2 neutral
int roller_state = 0; //0 nom, 1 spit, 2 neutral

void setMotors(int left, int right);
void setLift(int lift);
task arm();
task drive();
task button();

void pre_auton()
{
  bStopTasksBetweenModes = true;

}

task autonomous()
{

	AutonomousCodePlaceholderForTesting();  // Remove this function call once you have "real" code.


}

task usercontrol()
{
	while (true)
	{
		StartTask(arm);
		StartTask(drive);
		StartTask(button);
	}
}

task arm(){
	switch(lift_state){
		case 0:
			setLift(lift_rates[0]);
			break;
		case 1:
			setLift(lift_rates[1]);
			break;
		case 2:
			setLift(lift_rates[2]);
			break;
	}

	switch(roller_state){
		case 0:
			motor[LeftRoller] = 127;
			motor[RightRoller] = 127;
			break;
		case 1:
			motor[LeftRoller] = -127;
			motor[RightRoller] = -127;
			break;
		case 2:
			motor[LeftRoller] = 0;
			motor[RightRoller] = 0;
			break;
	}
}

task drive(){
	setMotors(vexRT[Ch3], vexRT[Ch2]);
}

task button(){
	if(vexRT[Btn5U] == 1) lift_state = 0;
	else if(vexRT[Btn5D] == 1) lift_state = 1;
	else lift_state = 2;

	if(vexRT[Btn6U] == 1) roller_state = 0;
	else if(vexRT[Btn6D] == 1) roller_state = 1;
	else roller_state = 2;

}

void setMotors(int left, int right){
	motor[LeftDrive1] = left;
	motor[LeftDrive2] = left;
	motor[RightDrive1] = right;
	motor[RightDrive2] = right;
}


void setLift(int lift){
	motor[LeftLift1] = lift;
	motor[LeftLift2] = lift;
	motor[RightLift1] = lift;
	motor[RightLift2] = lift;
}
