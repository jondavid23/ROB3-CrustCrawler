/*
    This code is being developed to Control
    a crustcrawler manipulator with emg signals.

*/

// Include the necessary ros packages
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>

// Include Arduino to make it possible to build it outside of the Arduino IDE
#include <Arduino.h>
#include "Dynamixel_Serial.h"
#include <SoftwareSerial.h>

#include <math.h>

// Define parameters
#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set too (57600)
#define LED13 0x0D
#define CW_LIMIT_ANGLE 0x001        // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
#define CCW_LIMIT_ANGLE 0xFFF       // Highest anit-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode

// Set the serial pins
SoftwareSerial mySerial( 10 , 11 );    // RX, TX

// Input is X, Y, Z

double invKin(double X, double Y, double Z){
  double base = 17.66;
  double link1 = 21.98;
  double link2 = 26.428;

  // Vector used in q1
  double V1[3] = {0, 0, 1};
  // Second vector used in q1
  double V2[3] = {X, Y, Z-base};
  // Dot product in q1
  double C;
  for(int i=0; i<3; i++){
    C += V1[i]+V2[i];
  }
  // The square root thing
  double S = sqrt((pow(X, 2)+pow(Y, 2)+pow((Z-base), 2)));
  // Deriving the second ange through q1 and q2
  double q1 = (pi/2)-acos(C/S);
  double q2 = acos((pow(link1, 2)-pow(link2, 2)+pow(S, 2)));
  // Deriving the third angle
  double q3 = acos((pow(link2, 2)+pow(link1, 2)-pow(S, 2)));
  // Deriving the first angle
  double theta1 = atan2(Y, X)
  // Deriving the second angle
  double theta2 = q1+q2;
  // Deriving the third angle
  double theta3 = q3-pi;

  // Find the angles in degrees
  double ang1 = (theta1*180)/pi+90;
  double ang2 = (theta2*180)/pi+90;
  double ang3 = (theta3*180)/pi+90;

  double pos[3];
  pos[0] = map(ang1, 0, 360, 0, 4095);
  pos[1] = map(ang2, 0, 360, 0, 4095);
  pos[2] = map(ang3, 0, 360, 0, 4095);

  return(pos);
}




void emg_move_pos(const std_msgs::UInt16MultiArray& emg){
    int emg_array[8];

    for(int i=0; i<8; i++){
      emg_array[i] = emg.data[i];
    }
    // Now we have the EMG data, lets convert them into x,y,z cartesian coordinates
    // EMG 2 is right hand outwards, that is if EMG is highest value change substract it from y


    }

// This function(moveEmg) was used to move the robot by setting goal positions with the
// EMG data coming in, when there were multiple joints being controlled there
// was a lot of cross talk, the jooints were not easilly controlled
void moveEmg(){
  // Move right EMG 2

  // EMG 2 is right (+) and EMG 6 is left (-)
    int emgDiff1 = emg_array[2]-emg_array[6];
    int currPos1 = getpos(1);
    int goalPos1 = currPos1 + emgDiff1;
    // Serial.println(goalPos1);

    int emgDiff2 = (emg_array[3]-emg_array[1]);
    int currPos2 = getpos(2);
    int goalPos2 = currPos2+emgDiff2;
    // Serial.println(goalPos2);
    int goalPos5, goalPos4;
    if(emg_array[0] && emg_array[1] > 500){
      int emgDiff4 = (emg_array[0]-emg_array[1]);
      int currPos4 = getpos(4);
      goalPos4 = currPos4 + emgDiff4;
      goalPos5 = getpos(5) - emgDiff4;
    }
    // Serial.println(goalPos2);

    // Move left EMG 6

    // // Move up EMG 3

    // // Move down EMG 1

    // // Close / open gripper
    // int goalPos5;
    // if(emg_array[0] && emg_array[1] > 500){
    //   int goalDeg5 = map(emg_array[1], 0, 2048, 0, 360);
    //   goalPos5 = map(goalDeg5, 0, 360, 0, 4095)+1000;
    // }
    //
    move(goalPos1, 3300, 1500 , 2045, 2045);
}


// Create a nodehandle for our subscriber
ros::NodeHandle  nh;
ros::Subscriber<std_msgs::UInt16MultiArray> sub("emg_data/mat_emg", emg_move_pos);

void setup(){
  // Initialise the node and subscriber
  nh.initNode();
  nh.subscribe(sub);

  Serial.flush();                                       // Clear the serial buffer of garbage data before running the code.
  mySerial.begin(SERVO_SET_Baudrate);                   // We now need to set Ardiuno to the new Baudrate speed 115200
  Serial.begin(57600);                                  // Start serial communication on baudrate 57600
  Dynamixel.begin(mySerial);                            // Calling mySerial function which sets 10 pin as the 2nd RX serial pin, and sets pin 11 as the 2nd TX serial pin
  Dynamixel.setDirectionPin(SERVO_ControlPin);
  // Turn on holding torque.
  set_torque(true);
  // Set the Profile acceleration.
  set_profVel(10);
  // Set the Profile velocity.
  set_profAcc(10);
}

void serial_setup(int BAUD){

}

void set_profVel(int ID){
  Dynamixel.setProfileVelocity(0x01, ID);  //Set the Profile Velocity for each servo. (max. is 1023)
  Dynamixel.setProfileVelocity(0x02, ID);  //Set the Profile Velocity for each servo. (max. is 1023)
  Dynamixel.setProfileVelocity(0x03, ID);  //Set the Profile Velocity for each servo. (max. is 1023)
}

void set_profAcc(int ID){
  Dynamixel.setProfileAcceleration(0x01, ID);  //Set the Profile Acceleration for each servo. (max. is 32767)
  Dynamixel.setProfileAcceleration(0x02, ID);  //Set the Profile Acceleration for each servo. (max. is 32767)
  Dynamixel.setProfileAcceleration(0x03, ID);  //Set the Profile Acceleration for each servo. (max. is 32767)
}

void set_torque(bool OnOff){
  Dynamixel.setHoldingTorque(0x01, OnOff);               //Turn on hold torque on servo 1
  Dynamixel.setHoldingTorque(0x02, OnOff);               //Turn on hold torque on servo 2
  Dynamixel.setHoldingTorque(0x03, OnOff);               //Turn on hold torque on servo 3
  Dynamixel.setHoldingTorque(0x04, OnOff);               //Turn on hold torque on servo 4
  Dynamixel.setHoldingTorque(0x05, OnOff);               //Turn on hold torque on servo 5
}

void end_serial(){
 Dynamixel.end();
 set_torque(false);
}

int getpos(int ID){
  return ( Dynamixel.getPosition(ID));
}

void move(int ID1, int ID2, int ID3, int ID4, int ID5){
  Dynamixel.setNGoalPositions(ID1,ID2,ID3,ID4,ID5);
  // TODO: add function to wait until finished
}

int avgArr(int arr[5]){
  int n,sum,avg;
  n = 5;
    for (int i = 0; i<n; i++){
        sum += arr[i];
    }
  avg = sum/n;
  return avg;
}

void pendulum(int goal1, int goal2, int goal3, int goal4, int goal5){
  int current_pos[5];
  int target_1[5] = {goal1,goal2,goal3,goal4,goal5};
  int target_2[5] = {goal1 + 500,goal2 + 500,goal3 + 500,goal4,goal5};
  // Get the current positions
  current_pos[0] = getpos(1);
  current_pos[1] = getpos(2);
  current_pos[2] = getpos(3);
  current_pos[3] = getpos(4);
  current_pos[4] = getpos(5);

  int loop = 1;
  int avg1, avg2;

  while (loop > 0){
    avg1 = avgArr(current_pos);
    avg2 = avgArr(target_1);
    Serial.println(avg1);
    Serial.println(avg2);
    move(target_1[0],target_1[1],target_1[2],target_1[3],target_1[4]);
    if (getpos(1) == current_pos[0] && getpos(2) == current_pos[1] && getpos(3)){
      loop = 0;
    }
  }
  move(target_2[0],target_2[1],target_2[2],target_2[3],target_2[4]);
  getpos(1);
  getpos(2);
  getpos(3);
  getpos(4);
  getpos(5);
  Serial.println("hej");

}

void loop(){
  nh.spinOnce();
  delay(1);
  //pendulum(1500, 2045, 2045, 2045, 2045);
  // end_serial();
}
