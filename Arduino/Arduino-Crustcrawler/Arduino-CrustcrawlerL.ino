/*
    This code is being developed to control a CrustCrawler manipulator with predetermined exercises. 
	The speed of the movement for the CrustCrawler will depend on the sEMG signals received from the 
	MYO Armband. A current controlled system will be used to determind the speed.
*/
/*********************
For Debugging
*********************/
//#define MYDEBUG
//#ifdef MYDEBUG
//#endif MYDEBUG

/*______________Include libraries______________*/
//Include math library to insure kinematic calculations
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

/*______________Define Parameters______________*/
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
#define PLAY 5
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

/*______________Global Variables______________*/
int button_pressed;
boolean power = ON;
double x = 48.4, y = 0, z = 17.7; 	//Setting up the cartesian coordinates for initial position
double _x,_y,_z;
int emg_array[8];

/*______________Serial Pins______________*/ 
IRrecv irrecv(receiver);
decode_results results;
SoftwareSerial mySerial( 10 , 11 );    // RX, TX

/*______________Functions______________*/ 
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

void print_dots(){
  delay(200); Serial.print("."); delay(200); Serial.print("."); delay(200); Serial.println(".");
}

void begin_serial(int vel,int acc){
  Serial.flush();                                       // Clear the serial buffer of garbage data before running the code.
  Serial.print("Starting up"); print_dots();
  mySerial.begin(SERVO_SET_Baudrate);                   // We now need to set Ardiuno to the new Baudrate speed 115200
  Dynamixel.begin(mySerial);                            // Calling mySerial function which sets 10 pin as the 2nd RX serial pin, and sets pin 11 as the 2nd TX serial pin
  Dynamixel.setDirectionPin(SERVO_ControlPin);
  set_torque(true);			// Turn on holding torque.
  set_profAcc(acc);			// Set the Profile acceleration.
  set_profVel(vel); 		// Set the Profile velocity.
  Serial.println("Moving CrustCrawler into initial position...");
  //int myPosition[5] = {0,0,0,0,0};
  //pendulum(myPosition);
  inverse_kine(x,y,z);
  Serial.println("Ready to begin procedure...");
}

void end_serial(){
 Serial.print("Turning off"); print_dots();
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
	if(arr[i]+100 > brr[i] && arr[i]-100 <brr[i]){
	  a = 1+a;
	}
	else{
	}
  }
  if(a == 5){
    for(int t = 0; t < 5; t++){
	  Serial.print("Servo_"); Serial.print(t+1); Serial.print(" ");
	  Serial.print("Desired: "); Serial.print(arr[t]); Serial.print("\n        ");
	  Serial.print("Current: "); Serial.println(brr[t]);
	}
	Serial.println("...Desired position reached...");
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

void inverse_kine(double x, double y, double z){
  double b = 17.66, L1 = 21.98, L2 = 26.428 ;
  double H = sqrt(pow(x,2)+pow(y,2)+pow((z-b),2));
  double q1 = PI / 2 -acos((z-b)/H);
  double q2 = acos((pow(L1,2)-pow(L2,2)+pow(H,2))/(2*L1*H));
  double q3 = acos((pow(L2,2)+pow(L1,2)-pow(H,2))/(2*L1*L2));

  //Deriving the first angle 
  double theta1 = atan2(y, x);
  // Deriving the second angle
  double theta2 = q1+q2;
  // Deriving the third angle
  double theta3 = q3 - PI;
  
  int ang1 = (int)(theta1*180)/PI;
  int ang2 = (int)(theta2*180)/PI;
  int ang3 = (int)(theta3*180)/PI;

  #ifdef MYDEBUG
  Serial.print(" H. "); Serial.print(H);
  Serial.print(" q1. "); Serial.print(q1);
  Serial.print(" q2. "); Serial.print(q2);
  Serial.print(" q3. "); Serial.print(q3);
  Serial.print(" pi. "); Serial.println(PI,7);
  #endif MYDEBUG
  
  Serial.print("Angles: ");
  Serial.print(" 1. "); Serial.print(ang1);
  Serial.print(" 2. "); Serial.print(ang2);
  Serial.print(" 3. "); Serial.println(ang3);
  
  int positionS[5] = {ang1,ang2,ang3,0,0};
  pendulum(positionS);

}

int translateIR(){ // takes action based on IR code received
  Serial.print("You pressed: ");
  switch(results.value){
  case 0xFFA25D: Serial.println("POWER");  button_pressed = POWER; 
  power = !power;
  if(power == false){ 
    end_serial();
  }
  else if(power == true){
	Serial.println("Turning on again");
    begin_serial(10,10);
  }
  break;
  case 0xFF629D: Serial.println("VOL+"); button_pressed = VOLP; 
  x += 5;
  break;
  case 0xFFE21D: Serial.println("FUNC/STOP"); button_pressed = FUNC; 
  x = 48.4, y = 0, z = 17.7;
  break;
  case 0xFF22DD: Serial.println("LEFT"); button_pressed = LEFT;  
  y += 5;
  break;
  case 0xFF02FD: Serial.println("PLAY/PAUSE"); button_pressed = PLAY; 
  Serial.println("Current Selected Position:");
  Serial.print("x = "); Serial.print(x); Serial.print(", "); Serial.print("y = "); Serial.print(y); Serial.print(", "); Serial.print("z = "); Serial.println(z); 
  inverse_kine(x,y,z);
  break;
  case 0xFFC23D: Serial.println("RIGHT"); button_pressed = RIGHT; 
  y -= 5;
  break;
  case 0xFFE01F: Serial.println("DOWN"); button_pressed = DOWN; 
  z -= 5;
  break;
  case 0xFFA857: Serial.println("VOL-"); button_pressed = VOLM; 
  x -= 5;
  break;
  case 0xFF906F: Serial.println("UP"); button_pressed = UP; 
  z += 5;
  break;
  case 0xFF6897: Serial.println("0"); button_pressed = ZERO; 
  
  break;
  case 0xFF9867: Serial.println("EQ"); button_pressed = EQ; 
  
  break;
  case 0xFFB04F: Serial.println("REPEAT"); button_pressed = REPEAT; 
  
  break;
  case 0xFF30CF: Serial.println("1"); button_pressed = ONE; 
  Serial.println("Exercise 1. have been selected."); 
  Serial.print("Please move your arm into a position that mirrors the arm of the CrustCrawler.");
  Serial.print("");
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
  Serial.println("Awaiting new order...");
  return button_pressed;
} 
  
void Callback(const std_msgs::UInt16MultiArray& emg){
    for(int i=0; i<8; i++){
      emg_array[i] = emg.data[i];
    }
}
/*______________Initiate ROS NodeHandler and Subscriber______________*/ 
ros::NodeHandle nh;
ros::Subscriber<std_msgs::UInt16MultiArray> sub("emg_data/mat_emg", Callback);

/*______________Setup______________*/ 
void setup(){
  nh.initNode();
  nh.subscribe(sub);
  Serial.begin(57600);                                  // Start serial communication on baudrate 57600
  irrecv.enableIRIn();
  begin_serial(10,10);
}
/*______________Loop______________*/ 
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
