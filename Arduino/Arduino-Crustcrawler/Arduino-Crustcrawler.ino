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

// Define parameters
#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set too (57600)
#define LED13 0x0D
#define CW_LIMIT_ANGLE 0x001        // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
#define CCW_LIMIT_ANGLE 0xFFF       // Highest anit-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode

// Set the serial pins
SoftwareSerial mySerial( 10 , 11 );    // RX, TX

void emg_move_pos(const std_msgs::UInt16MultiArray& emg){
    int emg_array[8];

    for(int i=0; i<sizeof(emg.data); i++){
      emg_array[i] = emg.data[i];
    }
 
}

// Create a nodehandle for our subscriber
ros::NodeHandle  nh;
ros::Subscriber<std_msgs::UInt16MultiArray> sub("mat_emg", emg_move_pos); // TODO: Add callback

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
    if (avg1 < avg2 + 5 && avg1 > avg2 -5){
      loop =0;
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
   pendulum(1500, 2045, 2045, 2045, 2045);
  // end_serial();
}
