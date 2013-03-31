#pragma config(Sensor, in1,    angle,               sensorPotentiometer)
#pragma config(Sensor, in2,    lineFollowerLEFT,    sensorLineFollower)
#pragma config(Sensor, in3,    lineFollowerCENTER,  sensorLineFollower)
#pragma config(Sensor, in4,    lineFollowerRIGHT,   sensorLineFollower)
#pragma config(Sensor, in5,    lineFollowerSIDE1,   sensorLineFollower)
#pragma config(Sensor, in6,    yaw,                 sensorGyro)
#pragma config(Sensor, in7,    lineFollowerSIDE2,   sensorLineFollower)
#pragma config(Sensor, in8,    mode_selector,       sensorPotentiometer)
#pragma config(Sensor, dgtl1,  conveyorFront,       sensorTouch)
#pragma config(Sensor, dgtl2,  conveyorBack,        sensorTouch)
#pragma config(Sensor, dgtl3,  leftEncoder,         sensorRotation)
#pragma config(Sensor, dgtl4,  rightEncoder,        sensorRotation)
#pragma config(Motor,  port1,           Left2Motor,    tmotorNormal, openLoop, reversed)
#pragma config(Motor,  port2,           Armleft1Motor, tmotorNormal, openLoop, reversed)
#pragma config(Motor,  port3,           Left1Motor,    tmotorNormal, openLoop)
#pragma config(Motor,  port4,           basket1Motor,  tmotorNormal, openLoop, reversed)
#pragma config(Motor,  port5,           conveyor1Motor, tmotorNormal, openLoop)
#pragma config(Motor,  port6,           basket2Motor,  tmotorNormal, openLoop)
#pragma config(Motor,  port7,           conveyor2Motor, tmotorNormal, openLoop, reversed)
#pragma config(Motor,  port8,           Right1Motor,   tmotorNormal, openLoop, reversed)
#pragma config(Motor,  port9,           Armleft2Motor, tmotorNormal, openLoop)
#pragma config(Motor,  port10,          Right2Motor,   tmotorNormal, openLoop)
#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"   //Main competition background code...do not modify!


//predefine the functions
void lift(int target); //core function, input a target degree, the arm will then lift to that angle and maintain that angle
void straight(int speed); //function to move straight based on the rotations of the encoders
void setMotors(int left, int right); //a lazy way to set motor speeds, less lines of code
void brake(int counter_force); //another lazy way, brakes the motor with a given counter force, then stops it.
void straight_distance(int distance, int speed_distance); //calls the straight function to go forward for a certain number of rotations
void turn(int clicks, int turn_speed); //uses encoder to turn a specific amount of clicks using the encoderTurn() function to ensure equal force
void line_follower(int speed); //follows a line, however isn't as flexible, must start out on the line
void encoderTurn(int speed); //a turning function that ensures that each wheels rotate the same amount
void arm(); //manages the different presets for the arm
void mode(); //manages which autonomous program to run
void autonomous1();
void setArmMotors(int speed);
void encoders();
void ready_preload();
task drive();
task conveyor_belt();


int lift_cycle; //global variable to store which arm preset to be used
int color; //1 = blue, -1 = red
int zone; //1 = interaction, -1 = isolation
int actual;

void pre_auton()
{
  encoders();
}

task autonomous()
{
  mode();
  autonomous1();
  motor[Armleft1Motor] = 0;
  motor[Armleft2Motor] = 0;

}

task usercontrol()
{
  encoders(); //initalizes the encoders
  SensorValue[yaw] = 0; //zeroes the gyro, not used though
  lift_cycle = 0; //sets the lift_cycle to 0
  lift(2020); //bring the lift to 2020 degrees

  while (true)
  {
    StartTask(drive); //starts the multitasking
    StartTask(conveyor_belt);
    arm(); //make sure the presets are being managed

    if(vexRT[Btn8L]) //functions run if button press
    {
      mode();
      autonomous1();
    }

    if(vexRT[Btn8U] == 1 || vexRT[Btn8UXmtr2] == 1) //readies the backpack for preloads
    {
      ready_preload();
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void encoders() //clear the encoders
{
  SensorValue[leftEncoder] = 0; //sets the left encoder to zero
  SensorValue[rightEncoder] = 0; //sets the right encoder to zero
}

void setMotors(int left, int right) //less code to write, allows to set drive motor speeds with one statement
{
  motor[Left1Motor] = left;
  motor[Left2Motor] = left;
  motor[Right1Motor] = right;
  motor[Right2Motor] = right;
}

void brake(int counter_force) //brakes with a variable counter force, then stops the motor
{
  setMotors(counter_force, counter_force); //uses the setMotor(); to set the drive motors
  wait1Msec(50); //allows this braking to take into effect
  setMotors(0, 0); //stops the motors
}

void straight(int speed) //goes straight based on the optical shaft encoder readings, input speed
{
  if(speed > 0) //if speed is positive, then it will do this function to go straight
  {
    if(SensorValue[leftEncoder] == SensorValue[rightEncoder]) //whilst the rotation is equal on both sides
    {
      setMotors(speed, speed); //set the input speed to the motors
    }
    else if(SensorValue[leftEncoder] > SensorValue[rightEncoder]) //however if leftside has more rotations than right side
    {
      setMotors(speed -30, speed); //slows down the left motor, whilst keeping the right motor at the target speed
    }
    else //anything else, right side has more rotations
    {
      setMotors(speed, speed - 30); //slow down right side and keeps the left side at target speed
    }
  }
  else //if speed is less than zero, or negative
  {
    if(SensorValue[leftEncoder] == SensorValue[rightEncoder]) //if left side has same number of rotations as right side
    {
      setMotors(speed, speed); //set both sides to the target speed
    }
    else if(SensorValue[leftEncoder] > SensorValue[rightEncoder]) //if left side has more rotations than right side
    {
      setMotors(speed + 30, speed); //slows down the left side, keeps right side at target speed (negative number, add positive to slow down
    }
    else //if right side has more rotations than left side
    {
      setMotors(speed, speed + 30); //slows down the right side and keeps the left side at target speed
    }
  }
}

void straight_distance(int distance, int speed_distance) //calls the straight task and runs it until it reaches a certain number of rotations
{
  encoders(); //first clears the encoder to start counting from zero

  int current = (SensorValue[leftEncoder] + SensorValue[rightEncoder]) / 2; //takes the current distance by averaging the values from the shaft encoders
  while(current < distance) //while the current clicks is less than the target clicks
  {
    straight(speed_distance); //the straight function is called with the target speed
    current = (SensorValue[leftEncoder] + SensorValue[rightEncoder]) / 2; //constantly re-writing current to keep it up to date
  }
  brake(0); //stops the motors
}

void encoderTurn(int speed) //Positive is clockwise, negative is counter clockwise, uses encoders to make sure they turn at the same rate
{
  int compensate; //how much to compensate the turn by

  if(speed>0) //if speed is positive, then compensate will be - 30
  {
    compensate = -30;
  }

  else //if speed is negative, then compensate will be postive 30
  {
    compensate = 30;
  }

  if(SensorValue[leftEncoder] == SensorValue[rightEncoder]) //while both of these are equal to each other
  {
    setMotors(speed, -1 * speed); //set motors to turn with one side reflected
  }

  else if(SensorValue[leftEncoder] > SensorValue[rightEncoder]) //if the left side turned more than the right side
  {
    setMotors(speed + compensate, -1 * speed); //slows down the left side
  }

  else //if right side has turned more
  {
    setMotors(speed, -1 * (speed + compensate)); //slows down the right side
  }
}

void turn(int clicks, int turn_speed) //turn based on optical shaft clicks
{
  encoders(); //clears the encoders

  while((SensorValue[leftEncoder] + SensorValue[rightEncoder])/2 < clicks) //averages the encoders and while that average is less than the target number of clicks
    encoderTurn(turn_speed); //calls encoderTurn(); to ensure a consistent turn at the target turn_speed
}

void line_follower(int speed) //line follower task, could be improved however this will suffice for now
{

  if(SensorValue[lineFollowerCENTER] < 500) //if the center linefollower detects a line
  {
    setMotors(speed, speed); //sets both sides to the target speed
  }

  else if(SensorValue[lineFollowerRIGHT] < 500) //if the right side detects a line, means its swerving to the right
  {
    setMotors(speed - 30, speed); //slows down the left side so right side catches up
  }

  else if(SensorValue[lineFollowerLEFT] < 500) //if the left side detects a line, means its swerving to the left
  {
    setMotors(speed, speed - 30); //slows down the right side to allow the left to catch up
  }
  else
  {
    setMotors(speed, speed);
  }
}

task drive() //core task that runs the drive motors based on joysticks positions
{
  setMotors(vexRT[Ch3], vexRT[Ch2]);
}

task conveyor_belt() //conveyor belt task and backpack
{
  while(vexRT[Btn5UXmtr2] == 1 || vexRT[Btn5U] == 1) //while this button is pressed
  {
    motor[conveyor1Motor] = 127; //engage the conveyor belt motors
    motor[conveyor2Motor] = 127;
    StartTask(drive); //keeps the drive task running
    if(SensorValue[conveyorFront] == 0) //makes sure the backpack doesn't extend while on the bottom, nor that it over extends itself
    {
      motor[basket1Motor] = 127; //if the conveyorFront limit switch isn't triggered, extends the backpack
      motor[basket2Motor] = 127;
    }
    else //once it is triggered
    {
      motor[basket1Motor] = 0; //stop the backpack from extending
      motor[basket2Motor] = 0;
    }
  }
  while(vexRT[Btn5DXmtr2] == 1 || vexRT[Btn5D] == 1) //while this button is pressed
  {
    motor[conveyor1Motor] = -127; //reverse the conveyor belt
    motor[conveyor2Motor] = -127;
    StartTask(drive); //keeps the robot drivable
    if(SensorValue[conveyorBack] == 0) //makes sure the backpack doesn't retract while on the bottom nor over retract.
    {
      motor[basket1Motor] = -127; //while the conveyorBack limit switch isn't triggered, retract it
      motor[basket2Motor] = -127;
    }
    else //once its triggered
    {
      motor[basket1Motor] = 0; //stop the backpack from retracting
      motor[basket2Motor] = 0;
    }
  }

  motor[conveyor1Motor] = 0; //if none of the buttons are being pressed, turn off all the motors
  motor[conveyor2Motor] = 0;
  motor[basket1Motor] = 0;
  motor[basket2Motor] = 0;
}

void lift(int target) //function to bring the arm to a certain degree, auto corrects itself so is always within 5 degrees of target.
{
  actual = SensorValue[angle]; //integer to store the angle of the arm, uses a variable for debugging purposes
  if(actual < target && target != 2020) //while the current angle of the arm is less than the target angle, also if target != to 2020
    //since this statements provides a counterforce, we don't want a counter force at the very bottom, thus this doesn't execute if its going to the bottom
  {
    ClearTimer(T2);
    while(actual < target && abs(target - actual) > 50 && vexRT[Btn8D] == 0  && vexRT[Btn8DXmtr2] == 0) //while the current is less than the target, as well as it exceed the threshold, (allows us to break the loop)
    {
      setArmMotors(127);  //moves the arm up
      actual = SensorValue[angle]; //keeps re-writing the current angle of the arm
      StartTask(drive);
      StartTask(conveyor_belt);
    }
    if(vexRT[Btn8D] == 1) //if the break button was pressed, sets the preset to 0 (2020)
      lift_cycle = 0;

    motor[Armleft1Motor] = 0; //adds a counter force to prevent the weight of the arm sinking the arm
    motor[Armleft2Motor] = 0;
  }
  else if(actual > target && abs(target - actual) > 50 && target != 2020) //however if the current angle is greater than the target, exceeds threshold, and is not equal to 2020
  {
    ClearTimer(T2);
    while(actual > target && abs(target - actual) > 50 && vexRT[Btn8D] == 0  && vexRT[Btn8DXmtr2] == 0) //runs a while loop
    {
      motor[Armleft1Motor] = -100; //moves the arm down
      motor[Armleft2Motor] = -100;
      actual = SensorValue[angle]; //re-writes the current angle of the arm
      StartTask(drive);
      StartTask(conveyor_belt);
    }
    if(vexRT[Btn8D] == 1) //if the break button was pressed
      lift_cycle = 0; //set the lift_cycle to preset 0, ground level
    motor[Armleft1Motor] = 0; //gives a counter fore
    motor[Armleft2Motor] = 0;
  }
  else if(target == 2020) //if the target is 2020
  {
    ClearTimer(T2);
    while(actual > target && abs(target - actual) > 45) //and while the current angle is greater than the target and exceeds the thresholds
    {
      motor[Armleft1Motor] = -120; //goes down
      motor[Armleft2Motor] = -120;
      actual = SensorValue[angle]; //rewrites the current angle of the arm
      StartTask(drive);
      StartTask(conveyor_belt);
    }
    motor[Armleft1Motor] = 0; //provides absolutely no counter force to give the motors a rest at ground level, also to reduce the annoying buzz of the motors
    motor[Armleft2Motor] = 0;
  }
}

void arm() //function in charge of managing the arm presets
{
  if(lift_cycle == -1) //ensures that the preset can never go below zero
    lift_cycle = 0;
  if(vexRT[Btn8D] == 1 || vexRT[Btn8DXmtr2] == 1) //a break button that goes directly to ground level
    lift_cycle = 0;

  if(vexRT[Btn6U] == 1 || vexRT[Btn6UXmtr2] == 1) //switches thru the presets
  {
    lift_cycle++; //adds one to the lift_cycle
    if(lift_cycle == 4) //ensures it doesn't go over three
      lift_cycle = 0;
  }
  if(vexRT[Btn6D] == 1 || vexRT[Btn6DXmtr2] == 1) //switches thru the presets
    lift_cycle--; //subtracts

  switch(lift_cycle) //switches the lift cycle to call different levels
  {
  case 0: //ground level
    lift(2020); //default zone
    break;

  case 1: //low goals
    lift(2650);
    break;

  case 2: //medium goals
    lift(3000);
    break;

  case 3: //talls goals
    lift(3500);
    break;
  }
}

void backpack()
{
  if(SensorValue[conveyorFront] == 0) //if the backpack isn't already extended
  {
    motor[basket1Motor] = 127; //extends the backpack all the way
    motor[basket2Motor] = 127;
  }

  if(vexRT[Btn8D] == 1 || vexRT[Btn8DXmtr2] == 1) //if the break was pressed
    lift_cycle = 0; //preset 0
  else if(SensorValue[conveyorFront] == 1)
  {
    motor[basket1Motor] = 0; //stops the basket motors
    motor[basket2Motor] = 0;
    lift_cycle = 1; //keeps the arm at the medium height
  }
}


void ready_preload() //gets the backpack up and ready to be preloaded
{
  backpack();
  if(actual < 2650) //while the current angle of the arm is less than the target angle, also if target != to 2020
    //since this statements provides a counterforce, we don't want a counter force at the very bottom, thus this doesn't execute if its going to the bottom
  {
    while(actual < 2650 && abs(2650 - actual) > 50 && vexRT[Btn8D] == 0  && vexRT[Btn8DXmtr2] == 0) //while the current is less than the target, as well as it exceed the threshold, (allows us to break the loop)
    {
      setArmMotors(127);  //moves the arm up
      actual = SensorValue[angle]; //keeps re-writing the current angle of the arm
      StartTask(drive);
      StartTask(conveyor_belt);
      backpack();
    }
    if(vexRT[Btn8D] == 1) //if the break button was pressed, sets the preset to 0 (2020)
      lift_cycle = 0;

    motor[Armleft1Motor] = 20; //adds a counter force to prevent the weight of the arm sinking the arm
    motor[Armleft2Motor] = 20;
  }
  else if(actual > 2650 && abs(2650 - actual) > 50) //however if the current angle is greater than the target, exceeds threshold, and is not equal to 2020
  {
    while(actual > 2650 && abs(2650 - actual) > 50 && vexRT[Btn8D] == 0  && vexRT[Btn8DXmtr2] == 0) //runs a while loop
    {
      motor[Armleft1Motor] = -100; //moves the arm down
      motor[Armleft2Motor] = -100;
      actual = SensorValue[angle]; //re-writes the current angle of the arm
      StartTask(drive);
      StartTask(conveyor_belt);
      backpack();
    }
    if(vexRT[Btn8D] == 1) //if the break button was pressed
      lift_cycle = 0; //set the lift_cycle to preset 0, ground level
    motor[Armleft1Motor] = 20; //gives a counter fore
    motor[Armleft2Motor] = 20;
  }

  while(SensorValue(conveyorFront) == 0)
    backpack();
}

void mode()
{
  if(SensorValue[mode_selector] <= 400) //blue isolation
  {
    color = 1;
    zone = -1;
  }
  else if(400 < SensorValue[mode_selector] && SensorValue[mode_selector] <= 1800) //blue interaction
  {
    color = 1;
    zone = 1;
  }
  else if(1800 < SensorValue[mode_selector] && SensorValue[mode_selector] <= 3100) //red interaction
  {
    color = -1;
    zone = 1;
  }
  else if(3100 < SensorValue[mode_selector]) //red isolation
  {
    color = -1;
    zone = -1;
  }
  else //just do nothing
  {
    color = 0;
    zone = 0;
  }
}

void setArmMotors(int speed)
{
  motor[Armleft1Motor] = speed;
  motor[Armleft2Motor] = speed;
}

void autonomous1() //autonomous function
{
  encoders();

  if(zone==-1) //if selected zone is isolate
  {
    while(SensorValue(conveyorFront) == 0) //stays put until the backpack is fully extended
    {
      motor[basket1Motor] = 127; //extends backpack
      motor[basket2Motor] = 127;
    }

    motor[conveyor1Motor] = 127; //starts the conveyor motors to pick up the objects
    motor[conveyor2Motor] = 127;

    straight_distance(610, 70); //goes forward for 420 counts, picking up all the balls and barrel in its path
    brake(-10); //brakes
    wait1Msec(700); //ensures that the last barrel is inside the system

    motor[conveyor1Motor] = 0; //stops the conveyor
    motor[conveyor2Motor] = 0;
    // FRONT LEFT CORNER

    turn(160, 80 * color); //turn counter or clockwise depending on the colour

    straight_distance(41, -80); //backs up a little bit
    brake(10); //brakes


  }
  else if(zone == 1) //interaction zone, needs further testing
  {

    ready_preload();

    straight_distance(500, -100);
    brake(10);
    straight_distance(550, 100);

    wait1Msec(5000);
    lift(2600);

    straight_distance(55, 60);

    motor[conveyor1Motor] = -127;
    motor[conveyor2Motor] = -127;

    wait1Msec(2000);

    straight_distance(150, -80);
    lift_cycle = 3;
  }
}
