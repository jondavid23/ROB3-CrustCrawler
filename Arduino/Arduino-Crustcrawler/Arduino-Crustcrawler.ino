/*
    This code is being developed to Control
    a crustcrawler manipulator with emg signals.

*/
/*********************
For Debugging
*********************/
//#define DEBUG
//#ifdef DEBUG
//#endif DEBUG

#include <math.h>
// Include the necessary ros packages
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>

// Include Arduino to make it possible to build it outside of the Arduino IDE
#include <Arduino.h>
#include "Dynamixel_Serial.h"
#include <SoftwareSerial.h>

// Include IRremote library to play around
#include <IRremote.h>

// Define parameters
#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set too (57600)
#define LED13 0x0D
#define CW_LIMIT_ANGLE 0x001        // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
#define CCW_LIMIT_ANGLE 0xFFF       // Highest anit-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode
#define receiver 12

//Define buttons on the IR remote
#define POWER 1
#define VOLP 2
#define FUNC 3
#define LEFT 4
#define PAUSE 5
#define RIGHT 6
#define DOWN 7
#define VOLM 8
#define UP 9
#define ZERO 10
#define EQ 11
#define REPEAT 12
#define ONE 13
#define TWO 14
#define THREE 15
#define FOUR 16
#define FIVE 17
#define SIX 18
#define SEVEN 19
#define EIGHT 20
#define NINE 21
#define ON true

//Global Variable for buttons
int button_pressed;
boolean power = ON;
//Setting up the cartesian coordinates for initial position
double x = 48.4, y = 0, z = 17.7;

// Set the serial pins
IRrecv irrecv(receiver);
decode_results results;
SoftwareSerial mySerial( 10 , 11 );    // RX, TX


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

void begin_serial(){
  Serial.flush();                                       // Clear the serial buffer of garbage data before running the code.
  Serial.print("Starting up");
  delay(100);
  Serial.print("."); delay(100); Serial.print("."); delay(100); Serial.println(".");
  mySerial.begin(SERVO_SET_Baudrate);                   // We now need to set Ardiuno to the new Baudrate speed 115200
  Dynamixel.begin(mySerial);                            // Calling mySerial function which sets 10 pin as the 2nd RX serial pin, and sets pin 11 as the 2nd TX serial pin
  Dynamixel.setDirectionPin(SERVO_ControlPin);
  // Turn on holding torque.
  set_torque(true);
  // Set the Profile acceleration.
  set_profAcc(10);
  // Set the Profile velocity.
  set_profVel(10);
  Serial.println("Moving CrustCrawler into initial position...");
  int myPosition[5] = {0,0,0,0,0};
  pendulum(myPosition);
  Serial.println("Ready to begin procedure...");
}

void end_serial(){
 Dynamixel.end();
 set_torque(false);
}

int getpos(int ID){
	unsigned char help = (unsigned char) ID;
  return ( Dynamixel.getPosition(help));
}

void move(int arr[]){
  Dynamixel.setNGoalPositions(arr[0],arr[1],arr[2],arr[3],arr[4]);
}

boolean positionCompare(int arr[],int brr[]){
  int a = 0 ;
  for(int i = 0 ; i < 5; i++){
    Serial.print("Iteration of i: "); Serial.println(i);
	Serial.print("Desired: "); Serial.println(arr[i]);
	Serial.print("Current: "); Serial.println(brr[i]);
	
	if(arr[i]+100 > brr[i] && arr[i]-100 <brr[i]){
	  a = 1+a;
	}
	else{
	}
	Serial.print("Value of a: "); Serial.println(a); Serial.println("");
  }
  Serial.println("");
  if(a == 5){
    return false ;
  }
  else{
    return true ;
  }
} 

void pendulum(int arr[]){
  arr[1] = (-(arr[1])+90);
  arr[2] = (-arr[2]);
  arr[4] = (-arr[4]);
  int desired_pos[5] = {deg(arr[0]),deg(arr[1]),deg(arr[2]),deg(arr[3]),deg(arr[4])};
  //Need to add something which checks if the postion is possible
  int current_pos[5];

  do{
	for(int i = 0 ; i < 5 ; i++){
	  current_pos[i] = getpos(i+1);
	}
    move(desired_pos);
	}
	while(positionCompare(desired_pos,current_pos));
}

int deg(int a){
  int b = map(a,-180,180,0,4095);
  return b;
}

void inverse_kine(int x, int y, int z){
  double b = 17.66, L1 = 21.98, L2 = 26.428 ;
  double q1 = PI / 0.2e1 - acos((z - b) * pow(x * x + y * y + pow(z - b, 0.2e1), -0.1e1 / 0.2e1));
  double q2 = acos((L1 * L1 - L2 * L2 + x * x + y * y + pow(z - b, 0.2e1)) / L1 * pow(x * x + y * y + pow(z - b, 0.2e1), -0.1e1 / 0.2e1) / 0.2e1);
  double q3 = acos((L2 * L2 + L1 * L1 - x * x - y * y - pow(z - b, 0.2e1)) / L1 / L2 / 0.2e1);

  //Deriving the first angle 
  double theta1 = atan2(y, x);
  // Deriving the second angle
  double theta2 = q1+q2;
  // Deriving the third angle
  double theta3 = q3 - PI;
  
  int ang1 = (int)(theta1*180)/PI;
  int ang2 = (int)(theta2*180)/PI;
  int ang3 = (int)(theta3*180)/PI;

  Serial.print("Angles: ");
  Serial.print(" 1. "); Serial.print(ang1);
  Serial.print(" 2. "); Serial.print(ang2);
  Serial.print(" 3. "); Serial.println(ang3);
  
  int positionS[5] = {ang1,ang2,ang3,0,0};
  pendulum(positionS);

}

int translateIR(){ // takes action based on IR code received
  switch(results.value){
  case 0xFFA25D: Serial.println("POWER");  button_pressed = POWER; 
  power = !ON;
  if(power == false){
    end_serial();
  }
  else if(power == true){
    begin_serial();
  }
  break;
  case 0xFF629D: Serial.println("VOL+"); button_pressed = VOLP; 
  x += 5;
  inverse_kine(x,y,z);
  break;
  case 0xFFE21D: Serial.println("FUNC/STOP"); button_pressed = FUNC; 
  
  break;
  case 0xFF22DD: Serial.println("LEFT"); button_pressed = LEFT;  
  y += 5;
  inverse_kine(x,y,z);
  break;
  case 0xFF02FD: Serial.println("PAUSE/START"); button_pressed = PAUSE; 
  
  break;
  case 0xFFC23D: Serial.println("RIGHT"); button_pressed = RIGHT; 
  y -= 5;
  inverse_kine(x,y,z);
  break;
  case 0xFFE01F: Serial.println("DOWN"); button_pressed = DOWN; 
  z -= 5;
  inverse_kine(x,y,z);
  break;
  case 0xFFA857: Serial.println("VOL-"); button_pressed = VOLM; 
  x -= 5;
  inverse_kine(x,y,z);
  break;
  case 0xFF906F: Serial.println("UP"); button_pressed = UP; 
  z += 5;
  inverse_kine(x,y,z);
  break;
  case 0xFF6897: Serial.println("0"); button_pressed = ZERO; 
  
  break;
  case 0xFF9867: Serial.println("EQ"); button_pressed = EQ; 
  
  break;
  case 0xFFB04F: Serial.println("REPEAT"); button_pressed = REPEAT; 
  
  break;
  case 0xFF30CF: Serial.println("1"); button_pressed = ONE; 
  
  break;
  case 0xFF18E7: Serial.println("2"); button_pressed = TWO; 
  
  break;
  case 0xFF7A85: Serial.println("3"); button_pressed = THREE; 
  
  break;
  case 0xFF10EF: Serial.println("4"); button_pressed = FOUR; 
  
  break;
  case 0xFF38C7: Serial.println("5"); button_pressed = FIVE; 
  
  break;
  case 0xFF5AA5: Serial.println("6"); button_pressed = SIX; 
  
  break;
  case 0xFF42BD: Serial.println("7"); button_pressed = SEVEN; 
  
  break;
  case 0xFF4AB5: Serial.println("8"); button_pressed = EIGHT; 
  
  break;
  case 0xFF52AD: Serial.println("9"); button_pressed = NINE; 
  
  break;
  case 0xFFFFFFFF: Serial.println("SAME AS BEFORE");
  
  break;  
  }// End Case
  Serial.println("...Desired position reached...");
  Serial.println("Awaiting new order...");
  return button_pressed;
} 
  
void Callback(const std_msgs::UInt16MultiArray& emg){
  int emg_array[8];
    for(int i=0; i<8; i++){
      emg_array[i] = emg.data[i];
    }
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::UInt16MultiArray> sub("emg_data/mat_emg", Callback);

void setup(){
  nh.initNode();
  nh.subscribe(sub);
  Serial.begin(57600);                                  // Start serial communication on baudrate 57600
  irrecv.enableIRIn();
  begin_serial();
}

void loop(){
  nh.spinOnce();
  
  if (irrecv.decode(&results)){ // have we received an IR signal?
	//Serial.println(results.value, HEX);
    int myVal = translateIR();
	//Serial.println(myVal);	
    irrecv.resume(); // receive the next value
  }
  
  delay(500); // Do not get immediate repeat
}
