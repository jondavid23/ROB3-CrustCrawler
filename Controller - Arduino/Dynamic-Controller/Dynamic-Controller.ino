/*
    This code is being developed to control a CrustCrawler manipulator with predetermined exercises.
	The speed of the movement for the CrustCrawler will depend on the sEMG signals received from the
	MYO Armband. A current based position controller is used to control the CrustCrawler.
*/

/*______________Include libraries______________*/
// Include Arduino to make it possible to build it outside of the Arduino IDE
#include <Arduino.h>
#include "Dynamixel_Serial.h"
#include <SoftwareSerial.h>
// Include the necessary ros packages
#include <math.h>
// Include ROS relevant headers
#include <ros.h>
#include <std_msgs/UInt8.h>

/*______________Define Parameters______________*/
#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set too (57600)
#define UNDEFINED 0

/*______________Global Variables______________*/
// Used for trajectory timing
int trajectory_time,temp_time,counter;
bool trajectory_done = false;
double before_time;
double current_time = 0;
double run_time = 0;
bool wait_on_gripper = false;
bool done = false;
bool exercise_done = false;

/*______________Serial Pins______________*/
SoftwareSerial mySerial( 10 , 11 );    // RX, TX

/*______________Functions______________*/
void setTorque(bool OnOff){
  Dynamixel.setHoldingTorque(0x01, OnOff);//Turn on hold torque on servo 1
  Dynamixel.setHoldingTorque(0x02, OnOff);//Turn on hold torque on servo 2
  Dynamixel.setHoldingTorque(0x03, OnOff);//Turn on hold torque on servo 3
  Dynamixel.setHoldingTorque(0x04, OnOff);//Turn on hold torque on servo 4
  Dynamixel.setHoldingTorque(0x05, OnOff);//Turn on hold torque on servo 5
}

int getPos(int ID){
  return ( Dynamixel.getPosition(ID));//Get the current position
}

int getVel(int ID){
  return ( Dynamixel.getVelocity(ID));//Get the current velocity
}

double deg2Step(double a){
  double b = map(a,-180,180,0,4095);//Convert degrees to steps
  return b;
}

double step2Deg(double a){
  double b = map(a,0,4095,-180,180);//Convert steps to degrees
  return b;
}

double rpm2Degs(double a){
  a *= 0.229;
  double b = a * 6;//Convert rpm to degrees
  return b;
}

double degs2Rpm(double a){
  double b = a / 0.01666666666666666666;//Convert degrees to rpm
  return b;
}

void move(double goal[]){
  for(int i = 0; i<3; i++){
    Dynamixel.setGoalPosition(i+1, goal[i]);//Move the servos to a set position
  }
}

class Trajectory
{
public:
  //Constructor
  Trajectory()
  {
  }//Trajectory

//***** Public functions *****
  // Set the exercise
  void setExercise(){
    switch(current_exercise){
      case 1: servo_one_goal = 0; servo_two_goal = 0; servo_three_goal = 70;//Select exercise #1
        break;
      case 2: servo_one_goal = 90; servo_two_goal = 90; servo_three_goal = -90;//Select exercise #2
        break;
      case 3: servo_one_goal = 90; servo_two_goal = -20; servo_three_goal = 22;//Select exercise #3
        break;
      case 4: servo_one_goal = 45; servo_two_goal = 45; servo_three_goal = -15;//Intermidiate point for exercise #3
        break;
      case 5: servo_one_goal = 0; servo_two_goal = -20; servo_three_goal = 22;//Intermidiate point for exercise #3
        break;
      case 0: servo_one_goal = 0; servo_two_goal = 0; servo_three_goal = 0;//Home position
        break;
    }
  }//Void setExercise
  //Reset the trajectory
  void reset(){
    trajectory_done = false;
    run_time = 0;
    done = false;
  }//Void reset

  //Check if the trajectory is done
  void posReached(){
    if(trajectory_done){
          if(current_exercise == 1 || current_exercise == 2 ){
            last_exercise = current_exercise;
            current_exercise = 0;
            setExercise();
            reset();
            return;
          }
          else if(current_exercise == 0){
            current_exercise = last_exercise;
            setExercise();
            reset();
            return;
          }
          else if(current_exercise == 3){
            if(!gripper_activated){
                wait_on_gripper = true;
            }
            if(!wait_on_gripper){
              gripper_activated = false;
              last_exercise = current_exercise;
              current_exercise = 4;
              setExercise();
              reset();
            }
          }
          else if (current_exercise == 4 && last_exercise == 3){
            last_exercise = current_exercise;
            current_exercise = 5;
            setExercise();
            reset();
          }
          else if(current_exercise == 4 && last_exercise == 5){
            last_exercise = current_exercise;
            current_exercise = 3;
            setExercise();
            reset();
          }
          else if (current_exercise == 5){
            if(!gripper_activated){
                wait_on_gripper = true;
            }
            if(!wait_on_gripper){
              gripper_activated = false;
              last_exercise = current_exercise;
              current_exercise = 4;
              setExercise();
              reset();
            }
          }
    }
  }//Void posReached

  // Get the trajectory
  void calcTra(){

    if(run_time == 0){
      before_time = millis();
    }
    while(run_time < tra_time){
       if(run_time != 0){
        before_time = current_time;
      }
      current_time = millis();
      run_time += ((current_time - before_time)/1000);
      double servo_one_position = servo_one_a0+(servo_one_a1*run_time)+(servo_one_a2*pow(run_time,2))+(servo_one_a3*pow(run_time,3));
      double servo_two_position = servo_two_a0+(servo_two_a1*run_time)+(servo_two_a2*pow(run_time,2))+(servo_two_a3*pow(run_time,3));
      double servo_three_position = servo_three_a0+(servo_three_a1*run_time)+(servo_three_a2*pow(run_time,2))+(servo_three_a3*pow(run_time,3));

      servo_pos_goal_one = deg2Step(servo_one_position);
      servo_pos_goal_two = deg2Step(-(fmod(servo_two_position - 90.0, 180)));
      servo_pos_goal_three = deg2Step(-(servo_three_position)); // was -

      Dynamixel.setGoalPosition(1, servo_pos_goal_one);
      Dynamixel.setGoalPosition(2, servo_pos_goal_two);
      Dynamixel.setGoalPosition(3, servo_pos_goal_three);
    }
      trajectory_done = true;
  }// void calcTra

  // Set the trajectory
  void planTra(int traT){
    // Read from the servos
    readServo();
    trajectory_done = false;
    tra_time = traT;

    //Changing the orientation configuration of the CrustCrawler
    current_Degree[1] = -(fmod(current_Degree[1]-90.0, 180));
    current_Degree[2] = -current_Degree[2];

    //Trajectory joint 1
    servo_one_a0 = current_Degree[0];
    servo_one_a1 = current_Degs[0];
    servo_one_a2 = (3/pow(traT,2))*(servo_one_goal-current_Degree[0])-(2/traT)*current_Degs[0];
    servo_one_a3 = (-2/pow(traT,3))*(servo_one_goal-current_Degree[0])+(1/(pow(traT,2)))*current_Degs[0];

    //Trajectory Joint 2
    servo_two_a0 = current_Degree[1];
    servo_two_a1 = current_Degs[1];
    servo_two_a2 = (3/pow(traT,2))*(servo_two_goal-current_Degree[1])-(2/traT)*current_Degs[1];
    servo_two_a3 = (-2/pow(traT,3))*(servo_two_goal-current_Degree[1])+(1/(pow(traT,2)))*current_Degs[1];

    //Trajectory Joint 3
    servo_three_a0 = current_Degree[2];
    servo_three_a1 = current_Degs[2];
    servo_three_a2 = (3/pow(traT,2))*(servo_three_goal-current_Degree[2])-(2/traT)*current_Degs[2];
    servo_three_a3 = (-2/pow(traT,3))*(servo_three_goal-current_Degree[2])+(1/(pow(traT,2)))*current_Degs[2];
  }// void planTra
  void reachableWorkspace(){
    readServo();
    if (current_Position[1]>3181 || current_Position[1]<909 ||current_Position[2]>3181 || current_Position[2]<909){
      protectiveStop();
    }
  }
  
  //Public variables
  bool gripper_activated = false;
  bool prot_stop = false;
  int current_exercise = 0;
  bool dynamic_boost = false;
private:
//***** Private functions *****
  // Read from the servos and convert to appropriate values
  void readServo(){
    // Fetch information from the Servos
    for(int i = 0; i < 3; i++){
      current_Position[i] = getPos(i+1);
      current_Degree[i] = step2Deg(current_Position[i]);
      current_Velocity[i] = getVel(i+1);
      current_Degs[i] = rpm2Degs(current_Velocity[i]);

    }
  }// void readServo
  void protectiveStop(){
    readServo();
    servo_pos_goal_one = current_Position[0];
    servo_pos_goal_two = current_Position[1];
    servo_pos_goal_three = current_Position[2];
    Dynamixel.setGoalPosition(1, servo_pos_goal_one);
    Dynamixel.setGoalPosition(2, servo_pos_goal_two);
    Dynamixel.setGoalPosition(3, servo_pos_goal_three);
    setTorque(false);
  }//Void protectiveStop

  void applyDynamics(){
    Dynamics(servo_one_position,servo_two_position,servo_three_position,servo_one_vel,servo_two_vel,servo_three_vel,servo_one_acc,servo_two_acc,servo_three_acc);
        if (!dynamic_boost){
          //Apllying the torque
          Dynamixel.setGoalCurrent(1,tau[0]/0.00336+400);
          Dynamixel .setGoalCurrent(2,tau[1]/0.00336+400);
          Dynamixel.setGoalCurrent(3,tau[2]/0.00336+400);
          dynamic_boost = true;
        }
        else {
        //Apllying the torque
          Dynamixel.setGoalCurrent(1,tau[0]+1/0.00336);
          Dynamixel.setGoalCurrent(2,tau[1]+1/0.00336);
          Dynamixel.setGoalCurrent(3,tau[2]+1/0.00336);
        }
    
    //Force control
    if (tau1 || tau2 || tau3 > 5){
     Dynamixel.end();
    }
  
  } 
  void Dynamics (double theta1,double theta2, double theta3, double thetad1, double thetad2, double thetad3, double thetadd1, double thetadd2, double thetadd3)
  {   //Gravitational constant
      g = 9.801;
      //Assigning values to variables
      L2 = 0.2198; L3 = 0.26428; M2 = 0.08823952; M3 = 0.12150869;
      I1xx = 0.00006223;I1yx = 0;I1zx = 0;I1xy = 0;I1yy = 0.00005344;I1zy = 0.00000123;I1xz = 0;I1yz = 0.00000123;I1zz = 0.00003775;
      I2xx = 0.00027038;I2yx = 0.00027038;I2zx = 0.00000325;I2xy = 0.00000002;I2yy = 0.00026195;I2yz = 0.00000156;I2xz = 0.00000325;I2yz = 0.00000156;I2zz = 0.00002254;
      I3xx =0.00028061;I3yx = 0;I3zx = -0.00000122;I3xy = 0;I3yy = 0.00024023;I3yz = 0;I3xz = -0.00000122;I3yz = 0;I3zz = 0.00006925;

      //Mass Matrix
      H11 = I1zz+(1/4)*M2*pow(L2,2)*pow(cos(theta2),2)*pow(sin(theta1),2)+(1/4)*M2*pow(L2,2)*pow(cos(theta2),2)*pow(cos(theta1),2)+I2zz+M3*pow((-L2*cos(theta2)*sin(theta1)-(1/2)*L3*cos(theta3+theta2)*sin(theta1)),2)+M3*pow((L2*cos(theta2)*cos(theta1)+(1/2)*L3*cos(theta3+theta2)*cos(theta1)),2)+I3zz;

      H12 = -(1/4)*M2*pow(L2,2)*sin(theta2)*pow(sin(theta1),2)*cos(theta2)-(1/4)*M2*pow(L2,2)*sin(theta2)*pow(cos(theta1),2)*cos(theta2)+(1/2)*cos(theta1)*I2xz+(1/2)*sin(theta1)*I2yz+(1/2)*I2zx*cos(theta1)+(1/2)*I2zy*sin(theta1)+M3*(L2*sin(theta2)*sin(theta1)+(1/2)*sin(theta1)*L3*sin(theta3))*(-L2*cos(theta2)*sin(theta1)-(1/2)*L3*cos(theta3+theta2)*sin(theta1))+M3*(-L2*sin(theta2)*cos(theta1)-(1/2)*cos(theta1)*L3*sin(theta3))*(L2*cos(theta2)*cos(theta1)+(1/2)*L3*cos(theta3+theta2)*cos(theta1))+(1/2)*cos(theta1)*I3xz+(1/2)*sin(theta1)*I3yz+(1/2)*I3zx*cos(theta1)+(1/2)*I3zy*sin(theta1);

      H13 = (1/2)*M3*sin(theta1)*L3*sin(theta3)*(-L2*cos(theta2)*sin(theta1)-(1/2)*L3*cos(theta3+theta2)*sin(theta1))-(1/2)*M3*cos(theta1)*L3*sin(theta3)*(L2*cos(theta2)*cos(theta1)+(1/2)*L3*cos(theta3+theta2)*cos(theta1))+(1/2)*cos(theta1)*I3xz+(1/2)*sin(theta1)*I3yz+(1/2)*I3zx*cos(theta1)+(1/2)*I3zy*sin(theta1);

      H21 = -(1/4)*M2*pow(L2,2)*sin(theta2)*pow(sin(theta1),2)*cos(theta2)-(1/4)*M2*pow(L2,2)*sin(theta2)*pow(cos(theta1),2)*cos(theta2)+(1/2)*cos(theta1)*I2xz+(1/2)*sin(theta1)*I2yz+(1/2)*I2zx*cos(theta1)+(1/2)*I2zy*sin(theta1)+M3*(L2*sin(theta2)*sin(theta1)+(1/2)*sin(theta1)*L3*sin(theta3))*(-L2*cos(theta2)*sin(theta1)-(1/2)*L3*cos(theta3+theta2)*sin(theta1))+M3*(-L2*sin(theta2)*cos(theta1)-(1/2)*cos(theta1)*L3*sin(theta3))*(L2*cos(theta2)*cos(theta1)+(1/2)*L3*cos(theta3+theta2)*cos(theta1))+(1/2)*cos(theta1)*I3xz+(1/2)*sin(theta1)*I3yz+(1/2)*I3zx*cos(theta1)+(1/2)*I3zy*sin(theta1);

      H22 = (1/4)*M2*pow(L2,2)*pow(sin(theta2),2)*pow(sin(theta1),2)+(1/4)*M2*pow(L2,2)*pow(sin(theta2),2)*pow(cos(theta1),2)+cos(theta1)*(I2xx*cos(theta1)+I2yx*sin(theta1))+sin(theta1)*(I2yx*cos(theta1)+I2yy*sin(theta1))+M3*pow((L2*sin(theta2)*sin(theta1)+(1/2)*sin(theta1)*L3*sin(theta3)),2)+M3*pow((-L2*sin(theta2)*cos(theta1)-(1/2)*cos(theta1)*L3*sin(theta3)),2)+cos(theta1)*(I3xx*cos(theta1)+I3yx*sin(theta1))+sin(theta1)*(I3yx*cos(theta1)+I3yy*sin(theta1));

      H23 = (1/2)*M3*sin(theta1)*L3*sin(theta3)*(L2*sin(theta2)*sin(theta1)+(1/2)*sin(theta1)*L3*sin(theta3))-(1/2)*M3*cos(theta1)*L3*sin(theta3)*(-L2*sin(theta2)*cos(theta1)-(1/2)*cos(theta1)*L3*sin(theta3))+cos(theta1)*(I3xx*cos(theta1)+I3yx*sin(theta1))+sin(theta1)*(I3yx*cos(theta1)+I3yy*sin(theta1));

      H31 = (1/2)*M3*sin(theta1)*L3*sin(theta3)*(-L2*cos(theta2)*sin(theta1)-(1/2)*L3*cos(theta3+theta2)*sin(theta1))-(1/2)*M3*cos(theta1)*L3*sin(theta3)*(L2*cos(theta2)*cos(theta1)+(1/2)*L3*cos(theta3+theta2)*cos(theta1))+(1/2)*cos(theta1)*I3xz+(1/2)*sin(theta1)*I3yz+(1/2)*I3zx*cos(theta1)+(1/2)*I3zy*sin(theta1);

      H32 = (1/2)*M3*sin(theta1)*L3*sin(theta3)*(L2*sin(theta2)*sin(theta1)+(1/2)*sin(theta1)*L3*sin(theta3))-(1/2)*M3*cos(theta1)*L3*sin(theta3)*(-L2*sin(theta2)*cos(theta1)-(1/2)*cos(theta1)*L3*sin(theta3))+cos(theta1)*(I3xx*cos(theta1)+I3yx*sin(theta1))+sin(theta1)*(I3yx*cos(theta1)+I3yy*sin(theta1));

      H33 = (1/4)*M3*pow(sin(theta1),2)*pow(L3,2)*pow(sin(theta3),2)+(1/4)*M3*pow(cos(theta1),2)*pow(L3,2)*pow(sin(theta3),2)+cos(theta1)*(I3xx*cos(theta1)+I3yx*sin(theta1))+sin(theta1)*(I3yx*cos(theta1)+I3yy*sin(theta1));
      //Velocity Vector
      V11 =-thetad3 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * pow(sin(theta3),2) * sin(theta2) * cos(theta2) / 0.2e1 + thetad3 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * cos(theta3) * pow(sin(theta2),2) * sin(theta3) / 0.2e1 + thetad3 * M3 * thetad2 * pow(sin(theta1),2) * L2 * pow(sin(theta2),2) * L3 * cos(theta3) / 0.2e1 - thetad3 * M3 * thetad1 * L3 * pow(sin(theta1),2) * sin(theta3) * pow(sin(theta2),2) * L2 - thetad3 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * sin(theta3) * pow(sin(theta2),2) * cos(theta3) / 0.2e1 + thetad3 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * pow(sin(theta3),2) * cos(theta2) * sin(theta2) / 0.2e1 - thetad3 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * pow(sin(theta3),2) * sin(theta2) * cos(theta2) / 0.2e1 + thetad3 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * cos(theta3) * pow(sin(theta2),2) * sin(theta3) / 0.2e1 + thetad3 * M3 * thetad2 * pow(sin(theta1),2) * L2 * pow(sin(theta2),2) * L3 * cos(theta3) / 0.2e1 - thetad3 * M3 * L3 * cos(theta3) * thetad2 * pow(sin(theta1),2) * L2 * cos(theta2) / 0.2e1 - thetad3 * M3 * L3 * cos(theta3) * thetad2 * pow(sin(theta1),2) * L2 * cos(theta2) / 0.2e1 - M3 * pow(L3,2)* pow(sin(theta3),2) * pow(thetad3,2)* pow(sin(theta1),2) * cos(theta2) / 0.4e1 + M3 * pow(L3,2)* pow(sin(theta3),2) * pow(thetad3,2)* pow(sin(theta1),2) * cos(theta2) / 0.4e1 + M3 * pow(L3,2)* pow(sin(theta3),2) * pow(thetad3,2)* pow(sin(theta1),2) * cos(theta2) / 0.4e1 - M3 * pow(L3,2)* pow(sin(theta3),2) * pow(thetad3,2)* pow(sin(theta1),2) * cos(theta2) / 0.4e1 + M3 * pow(L3,2)* pow(sin(theta3),2) * pow(thetad2,2)* pow(sin(theta1),2) * cos(theta2) / 0.4e1 + M3 * pow(L3,2)* pow(sin(theta3),2) * pow(thetad2,2)* pow(sin(theta1),2) * cos(theta2) / 0.4e1 + thetad2 * M3 * thetad1 * L2 * pow(sin(theta2),2) * pow(sin(theta1),2) * L3 * sin(theta3) - thetad2 * M3 * thetad1 * L3 * pow(sin(theta1),2) * sin(theta3) * pow(sin(theta2),2) * L2 - thetad2 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * sin(theta3) * pow(sin(theta2),2) * cos(theta3) / 0.2e1 + thetad2 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * pow(sin(theta3),2) * cos(theta2) * sin(theta2) / 0.2e1 - thetad2 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * pow(sin(theta3),2) * sin(theta2) * cos(theta2) / 0.2e1 + thetad2 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * cos(theta3) * pow(sin(theta2),2) * sin(theta3) / 0.2e1 + thetad2 * M3 * L3 * sin(theta3) * thetad3 * pow(sin(theta1),2) * L2 * sin(theta2) / 0.2e1 + 0.3e1 / 0.4e1 * thetad2 * M3 * pow(L3,2)* sin(theta3) * thetad3 * pow(sin(theta1),2) * cos(theta3) * sin(theta2) + thetad2 * M3 * thetad1 * L2 * pow(sin(theta2),2) * pow(sin(theta1),2) * L3 * sin(theta3) - thetad2 * M3 * thetad1 * L3 * pow(sin(theta1),2) * sin(theta3) * pow(sin(theta2),2) * L2 - thetad2 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * sin(theta3) * pow(sin(theta2),2) * cos(theta3) / 0.2e1 + thetad2 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * pow(sin(theta3),2) * cos(theta2) * sin(theta2) / 0.2e1 - thetad2 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * pow(sin(theta3),2) * sin(theta2) * cos(theta2) / 0.2e1 + thetad2 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * cos(theta3) * pow(sin(theta2),2) * sin(theta3) / 0.2e1 + thetad2 * M3 * L3 * sin(theta3) * thetad3 * pow(sin(theta1),2) * L2 * sin(theta2) / 0.2e1 + 0.3e1 / 0.4e1 * thetad2 * M3 * pow(L3,2)* sin(theta3) * thetad3 * pow(sin(theta1),2) * cos(theta3) * sin(theta2) - thetad3 * M3 * thetad1 * L3 * pow(sin(theta1),2) * sin(theta3) * pow(sin(theta2),2) * L2 - thetad3 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * sin(theta3) * pow(sin(theta2),2) * cos(theta3) / 0.2e1 + thetad3 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * pow(sin(theta3),2) * cos(theta2) * sin(theta2) / 0.2e1 + M3 * pow(thetad2,2)* pow(sin(theta1),2) * L2 * cos(theta2) * L3 * sin(theta3) * sin(theta2) + M3 * pow(thetad2,2)* pow(sin(theta1),2) * L2 * cos(theta2) * L3 * sin(theta3) * sin(theta2) - thetad2 * M2 * pow(L2,2)* cos(theta2) * pow(sin(theta1),2) * thetad1 * sin(theta2) / 0.2e1 - thetad2 * M2 * pow(L2,2)* cos(theta2) * pow(sin(theta1),2) * thetad1 * sin(theta2) / 0.2e1 - 0.2e1 * thetad2 * M3 * thetad1 * pow(L2,2)* sin(theta2) * pow(sin(theta1),2) * cos(theta2) + thetad2 * M3 * pow(L3,2)* pow(sin(theta3),2) * thetad3 * pow(sin(theta1),2) * cos(theta2) / 0.2e1 - 0.2e1 * thetad2 * M3 * thetad1 * pow(L2,2)* sin(theta2) * pow(sin(theta1),2) * cos(theta2) + thetad2 * M3 * pow(L3,2)* pow(sin(theta3),2) * thetad3 * pow(sin(theta1),2) * cos(theta2) / 0.2e1 - M3 * pow(thetad2,2)* pow(sin(theta1),2) * L2 * pow(sin(theta2),2) * L3 * cos(theta3) / 0.2e1 + M3 * pow(thetad2,2)* pow(sin(theta1),2) * L2 * pow(sin(theta2),2) * L3 * cos(theta3) / 0.2e1 + M3 * L3 * sin(theta3) * pow(thetad2,2)* pow(sin(theta1),2) * L2 * sin(theta2) / 0.2e1 + M3 * pow(L3,2)* sin(theta3) * pow(thetad2,2)* pow(sin(theta1),2) * cos(theta3) * sin(theta2) / 0.4e1 - M3 * pow(thetad2,2)* pow(sin(theta1),2) * L2 * pow(sin(theta2),2) * L3 * cos(theta3) / 0.2e1 + M3 * pow(thetad2,2)* pow(sin(theta1),2) * L2 * pow(sin(theta2),2) * L3 * cos(theta3) / 0.2e1 + M3 * L3 * sin(theta3) * pow(thetad2,2)* pow(sin(theta1),2) * L2 * sin(theta2) / 0.2e1 + M3 * pow(L3,2)* sin(theta3) * pow(thetad2,2)* pow(sin(theta1),2) * cos(theta3) * sin(theta2) / 0.4e1 - pow(thetad2,2)* sin(theta1) * I2yy * cos(theta1) + pow(thetad2,2)* sin(theta1) * I2xx * cos(theta1) + pow(thetad2,2)* sin(theta1) * I3xx * cos(theta1) + pow(thetad3,2)* sin(theta1) * I3xx * cos(theta1) - 0.2e1 * thetad2 * pow(sin(theta1),2) * I3yx * thetad3 + 0.2e1 * thetad2 * pow(sin(theta1),2) * I3yx * thetad3 - pow(thetad2,2)* sin(theta1) * I3yy * cos(theta1) - pow(thetad3,2)* sin(theta1) * I3yy * cos(theta1) - M2 * pow(L2,2)* pow(sin(theta2),2) * pow(sin(theta1),2) * pow(thetad2,2)/ 0.4e1 + M2 * pow(L2,2)* pow(sin(theta2),2) * pow(sin(theta1),2) * pow(thetad2,2)/ 0.4e1 - M3 * pow(thetad2,2)* pow(sin(theta1),2) * pow(L2,2)* pow(sin(theta2),2) + M3 * pow(thetad2,2)* pow(sin(theta1),2) * pow(L2,2)* pow(sin(theta2),2) - M3 * pow(thetad2,2)* pow(sin(theta1),2) * pow(L2,2)* pow(sin(theta2),2) + M3 * pow(thetad2,2)* pow(sin(theta1),2) * pow(L2,2)* pow(sin(theta2),2) - 0.2e1 * thetad2 * sin(theta1) * I3yy * thetad3 * cos(theta1) + 0.2e1 * thetad2 * sin(theta1) * I3xx * thetad3 * cos(theta1) - M2 * pow(L2,2)* pow(sin(theta2),2) * pow(sin(theta1),2) * pow(thetad2,2)/ 0.4e1 + M2 * pow(L2,2)* pow(sin(theta2),2) * pow(sin(theta1),2) * pow(thetad2,2)/ 0.4e1 - thetad3 * M3 * pow(L3,2)* pow(sin(theta3),2) * thetad2 * pow(sin(theta1),2) * cos(theta2) / 0.4e1 - thetad3 * M3 * pow(L3,2)* pow(sin(theta3),2) * thetad2 * pow(sin(theta1),2) * cos(theta2) / 0.4e1 + M3 * pow(L3,2)* sin(theta3) * pow(thetad3,2)* pow(sin(theta1),2) * cos(theta3) * sin(theta2) / 0.2e1 + M3 * pow(L3,2)* sin(theta3) * pow(thetad3,2)* pow(sin(theta1),2) * cos(theta3) * sin(theta2) / 0.2e1 - M3 * L3 * cos(theta3) * pow(thetad3,2)* pow(sin(theta1),2) * L2 * cos(theta2) / 0.2e1 - M3 * L3 * cos(theta3) * pow(thetad3,2)* pow(sin(theta1),2) * L2 * cos(theta2) / 0.2e1 + pow(thetad2,2)* pow(sin(theta1),2) * I3yx + pow(thetad3,2)* pow(sin(theta1),2) * I3yx - pow(thetad2,2)* pow(sin(theta1),2) * I2yx - pow(thetad2,2)* pow(sin(theta1),2) * I3yx - pow(thetad3,2)* pow(sin(theta1),2) * I3yx + pow(thetad2,2)* pow(sin(theta1),2) * I2yx - 0.2e1 * thetad2 * M3 * thetad1 * L2 * sin(theta2) * pow(sin(theta1),2) * L3 * cos(theta3) * cos(theta2) - 0.2e1 * thetad2 * M3 * thetad1 * L2 * sin(theta2) * pow(sin(theta1),2) * L3 * cos(theta3) * cos(theta2) + thetad3 * M3 * thetad2 * pow(sin(theta1),2) * L2 * cos(theta2) * L3 * sin(theta3) * sin(theta2) / 0.2e1 - thetad3 * M3 * thetad1 * L2 * sin(theta2) * pow(sin(theta1),2) * L3 * cos(theta3) * cos(theta2) + thetad3 * M3 * thetad2 * pow(sin(theta1),2) * L2 * cos(theta2) * L3 * sin(theta3) * sin(theta2) / 0.2e1 - thetad3 * M3 * thetad1 * L2 * sin(theta2) * pow(sin(theta1),2) * L3 * cos(theta3) * cos(theta2);
      V21 = M3 * pow(L3,2)* sin(theta3) * pow(thetad3,2)* pow(sin(theta1),2) * cos(theta3) / 0.2e1 - sin(theta1) * I2xz * pow(thetad1,2)/ 0.2e1 + cos(theta1) * I2yz * pow(thetad1,2)/ 0.2e1 - pow(thetad1,2)* I2zx * sin(theta1) / 0.2e1 + pow(thetad1,2)* I2zy * cos(theta1) / 0.2e1 - sin(theta1) * I3xz * pow(thetad1,2)/ 0.2e1 + cos(theta1) * I3yz * pow(thetad1,2)/ 0.2e1 - pow(thetad1,2)* I3zx * sin(theta1) / 0.2e1 + pow(thetad1,2)* I3zy * cos(theta1) / 0.2e1 + thetad2 * M3 * pow(L3,2)* sin(theta3) * thetad3 * pow(sin(theta1),2) * cos(theta3) / 0.2e1 + thetad2 * M3 * pow(L3,2)* sin(theta3) * thetad3 * pow(sin(theta1),2) * cos(theta3) / 0.2e1 + M3 * pow(thetad2,2)* pow(sin(theta1),2) * L2 * cos(theta2) * L3 * sin(theta3) / 0.2e1 + M3 * pow(thetad2,2)* pow(sin(theta1),2) * L2 * cos(theta2) * L3 * sin(theta3) / 0.2e1 + M2 * pow(L2,2)* sin(theta2) * pow(sin(theta1),2) * pow(thetad2,2)* cos(theta2) / 0.4e1 + M2 * pow(L2,2)* sin(theta2) * pow(sin(theta1),2) * pow(thetad2,2)* cos(theta2) / 0.4e1 + M3 * pow(thetad2,2)* pow(sin(theta1),2) * pow(L2,2)* cos(theta2) * sin(theta2) + M3 * pow(thetad2,2)* pow(sin(theta1),2) * pow(L2,2)* cos(theta2) * sin(theta2) + M2 * pow(thetad1,2)* pow(L2,2)* cos(theta2) * pow(sin(theta1),2) * sin(theta2) / 0.4e1 + M2 * pow(thetad1,2)* pow(L2,2)* cos(theta2) * pow(sin(theta1),2) * sin(theta2) / 0.4e1 + M3 * pow(thetad1,2)* pow(L2,2)* cos(theta2) * pow(sin(theta1),2) * sin(theta2) + M3 * pow(thetad1,2)* pow(L2,2)* cos(theta2) * pow(sin(theta1),2) * sin(theta2) + thetad3 * M3 * thetad1 * L3 * pow(sin(theta1),2) * sin(theta3) * cos(theta2) * L2 * sin(theta2) / 0.2e1 + thetad3 * M3 * thetad1 * L3 * pow(sin(theta1),2) * sin(theta3) * cos(theta2) * L2 * sin(theta2) / 0.2e1 + thetad3 * M3 * thetad1 * L3 * pow(sin(theta1),2) * cos(theta3) * pow(sin(theta2),2) * L2 / 0.2e1 + thetad3 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * cos(theta3) * sin(theta2) * sin(theta3) / 0.4e1 + thetad3 * M3 * thetad1 * L3 * pow(sin(theta1),2) * cos(theta3) * pow(sin(theta2),2) * L2 / 0.2e1 + thetad3 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * cos(theta3) * sin(theta2) * sin(theta3) / 0.4e1 + thetad3 * M3 * L3 * cos(theta3) * thetad2 * pow(sin(theta1),2) * L2 * sin(theta2) - thetad3 * M3 * pow(sin(theta1),2) * L3 * cos(theta3) * thetad1 * L2 * cos(theta2) / 0.2e1 + thetad3 * M3 * L3 * cos(theta3) * thetad2 * pow(sin(theta1),2) * L2 * sin(theta2) - thetad3 * M3 * pow(sin(theta1),2) * L3 * cos(theta3) * thetad1 * L2 * cos(theta2) / 0.2e1 + M3 * pow(thetad1,2)* L2 * cos(theta2) * pow(sin(theta1),2) * L3 * cos(theta3) * sin(theta2) - M3 * L3 * sin(theta3) * thetad3 * pow(sin(theta1),2) * thetad1 * L2 * sin(theta2) / 0.2e1 + M3 * pow(thetad1,2)* L2 * cos(theta2) * pow(sin(theta1),2) * L3 * cos(theta3) * sin(theta2) - M3 * L3 * sin(theta3) * thetad3 * pow(sin(theta1),2) * thetad1 * L2 * sin(theta2) / 0.2e1 + M3 * pow(L3,2)* sin(theta3) * pow(thetad3,2)* pow(sin(theta1),2) * cos(theta3) / 0.2e1 + 0.2e1 * thetad1 * cos(theta1) * thetad2 * sin(theta1) * I3yy + 0.2e1 * thetad1 * cos(theta1) * I3yy * thetad3 * sin(theta1) - 0.2e1 * thetad1 * thetad2 * cos(theta1) * I2xx * sin(theta1) + 0.2e1 * thetad1 * thetad2 * cos(theta1) * I2yy * sin(theta1) - 0.2e1 * thetad1 * cos(theta1) * thetad2 * sin(theta1) * I3xx - 0.2e1 * thetad1 * cos(theta1) * I3xx * thetad3 * sin(theta1) + 0.2e1 * thetad1 * thetad2 * pow(sin(theta1),2) * I3yx + 0.2e1 * thetad1 * thetad2 * pow(sin(theta1),2) * I2yx - 0.2e1 * thetad1 * thetad2 * pow(sin(theta1),2) * I2yx + 0.2e1 * thetad1 * I3yx * thetad3 * pow(sin(theta1),2) - 0.2e1 * thetad1 * I3yx * thetad3 * pow(sin(theta1),2) - 0.2e1 * thetad1 * thetad2 * pow(sin(theta1),2) * I3yx - thetad3 * M3 * pow(sin(theta1),2) * pow(L3,2)* pow(sin(theta3),2) * thetad1 * cos(theta2) / 0.4e1 - thetad3 * M3 * pow(sin(theta1),2) * pow(L3,2)* pow(sin(theta3),2) * thetad1 * cos(theta2) / 0.4e1 + M3 * L3 * cos(theta3) * pow(thetad3,2)* pow(sin(theta1),2) * L2 * sin(theta2) / 0.2e1 + M3 * L3 * cos(theta3) * pow(thetad3,2)* pow(sin(theta1),2) * L2 * sin(theta2) / 0.2e1 + M3 * pow(thetad1,2)* L2 * pow(sin(theta2),2) * pow(sin(theta1),2) * L3 * sin(theta3) / 0.2e1 + M3 * pow(thetad1,2)* pow(L3,2)* pow(sin(theta1),2) * cos(theta3) * pow(sin(theta2),2) * sin(theta3) / 0.4e1 + M3 * pow(thetad1,2)* pow(L3,2)* pow(sin(theta1),2) * pow(sin(theta3),2) * cos(theta2) * sin(theta2) / 0.4e1 - M3 * pow(thetad1,2)* L3 * pow(sin(theta1),2) * sin(theta3) * pow(sin(theta2),2) * L2 / 0.2e1 - M3 * pow(thetad1,2)* pow(L3,2)* pow(sin(theta1),2) * pow(sin(theta3),2) * sin(theta2) * cos(theta2) / 0.4e1 - M3 * pow(thetad1,2)* pow(L3,2)* pow(sin(theta1),2) * sin(theta3) * pow(sin(theta2),2) * cos(theta3) / 0.4e1 + M3 * pow(thetad1,2)* L2 * pow(sin(theta2),2) * pow(sin(theta1),2) * L3 * sin(theta3) / 0.2e1 + M3 * pow(thetad1,2)* pow(L3,2)* pow(sin(theta1),2) * cos(theta3) * pow(sin(theta2),2) * sin(theta3) / 0.4e1 + M3 * pow(thetad1,2)* pow(L3,2)* pow(sin(theta1),2) * pow(sin(theta3),2) * cos(theta2) * sin(theta2) / 0.4e1 - M3 * pow(thetad1,2)* L3 * pow(sin(theta1),2) * sin(theta3) * pow(sin(theta2),2) * L2 / 0.2e1 - M3 * pow(thetad1,2)* pow(L3,2)* pow(sin(theta1),2) * pow(sin(theta3),2) * sin(theta2) * cos(theta2) / 0.4e1 - M3 * pow(thetad1,2)* pow(L3,2)* pow(sin(theta1),2) * sin(theta3) * pow(sin(theta2),2) * cos(theta3) / 0.4e1;
      V31 = thetad2 * M3 * thetad1 * L2 * sin(theta2) * pow(sin(theta1),2) * L3 * sin(theta3) / 0.2e1 - thetad2 * M3 * thetad1 * L3 * pow(sin(theta1),2) * cos(theta3) * pow(sin(theta2),2) * L2 / 0.2e1 - thetad2 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * cos(theta3) * sin(theta2) * sin(theta3) / 0.4e1 + thetad2 * M3 * thetad1 * L2 * sin(theta2) * pow(sin(theta1),2) * L3 * sin(theta3) / 0.2e1 - thetad2 * M3 * thetad1 * L3 * pow(sin(theta1),2) * cos(theta3) * pow(sin(theta2),2) * L2 / 0.2e1 - thetad2 * M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * cos(theta3) * sin(theta2) * sin(theta3) / 0.4e1 + M3 * thetad1 * L2 * cos(theta2) * pow(sin(theta1),2) * L3 * cos(theta3) * thetad2 / 0.2e1 + M3 * thetad1 * L2 * cos(theta2) * pow(sin(theta1),2) * L3 * cos(theta3) * thetad2 / 0.2e1 - M3 * pow(thetad2,2)* pow(sin(theta1),2) * L2 * sin(theta2) * L3 * cos(theta3) / 0.2e1 + M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * pow(sin(theta3),2) * cos(theta2) * thetad2 / 0.4e1 - M3 * pow(L3,2)* sin(theta3) * pow(thetad2,2)* pow(sin(theta1),2) * cos(theta3) / 0.4e1 - M3 * pow(L3,2)* sin(theta3) * pow(thetad2,2)* pow(sin(theta1),2) * cos(theta3) / 0.4e1 + M3 * pow(L3,2)* sin(theta3) * pow(thetad3,2)* pow(sin(theta1),2) * cos(theta3) / 0.4e1 - sin(theta1) * I3xz * pow(thetad1,2)/ 0.2e1 + cos(theta1) * I3yz * pow(thetad1,2)/ 0.2e1 - pow(thetad1,2)* I3zx * sin(theta1) / 0.2e1 + pow(thetad1,2)* I3zy * cos(theta1) / 0.2e1 + M3 * pow(thetad2,2)* pow(sin(theta1),2) * L2 * cos(theta2) * L3 * sin(theta3) / 0.2e1 + M3 * pow(thetad2,2)* pow(sin(theta1),2) * L2 * cos(theta2) * L3 * sin(theta3) / 0.2e1 - M3 * pow(thetad2,2)* pow(sin(theta1),2) * L2 * sin(theta2) * L3 * cos(theta3) / 0.2e1 + M3 * thetad1 * pow(L3,2)* pow(sin(theta1),2) * pow(sin(theta3),2) * cos(theta2) * thetad2 / 0.4e1 - thetad2 * M3 * thetad1 * L3 * pow(sin(theta1),2) * sin(theta3) * cos(theta2) * L2 * sin(theta2) / 0.2e1 - thetad2 * M3 * thetad1 * L3 * pow(sin(theta1),2) * sin(theta3) * cos(theta2) * L2 * sin(theta2) / 0.2e1 + M3 * pow(thetad1,2)* L2 * cos(theta2) * pow(sin(theta1),2) * L3 * cos(theta3) * sin(theta2) / 0.2e1 + M3 * pow(thetad1,2)* L2 * cos(theta2) * pow(sin(theta1),2) * L3 * cos(theta3) * sin(theta2) / 0.2e1 + M3 * pow(L3,2)* sin(theta3) * pow(thetad3,2)* pow(sin(theta1),2) * cos(theta3) / 0.4e1 + 0.2e1 * thetad1 * cos(theta1) * thetad2 * sin(theta1) * I3yy + 0.2e1 * thetad1 * cos(theta1) * I3yy * thetad3 * sin(theta1) - 0.2e1 * thetad1 * cos(theta1) * thetad2 * sin(theta1) * I3xx - 0.2e1 * thetad1 * cos(theta1) * I3xx * thetad3 * sin(theta1) + 0.2e1 * thetad1 * thetad2 * pow(sin(theta1),2) * I3yx + 0.2e1 * thetad1 * I3yx * thetad3 * pow(sin(theta1),2) - 0.2e1 * thetad1 * I3yx * thetad3 * pow(sin(theta1),2) - 0.2e1 * thetad1 * thetad2 * pow(sin(theta1),2) * I3yx + M3 * pow(thetad1,2)* L2 * pow(sin(theta2),2) * pow(sin(theta1),2) * L3 * sin(theta3) / 0.2e1 + M3 * pow(thetad1,2)* pow(L3,2)* pow(sin(theta1),2) * cos(theta3) * pow(sin(theta2),2) * sin(theta3) / 0.4e1 + M3 * pow(thetad1,2)* pow(L3,2)* pow(sin(theta1),2) * pow(sin(theta3),2) * cos(theta2) * sin(theta2) / 0.4e1 - M3 * pow(thetad1,2)* pow(L3,2)* pow(sin(theta1),2) * pow(sin(theta3),2) * sin(theta2) * cos(theta2) / 0.4e1 - M3 * pow(thetad1,2)* pow(L3,2)* pow(sin(theta1),2) * sin(theta3) * pow(sin(theta2),2) * cos(theta3) / 0.4e1 + M3 * pow(thetad1,2)* L2 * pow(sin(theta2),2) * pow(sin(theta1),2) * L3 * sin(theta3) / 0.2e1 + M3 * pow(thetad1,2)* pow(L3,2)* pow(sin(theta1),2) * cos(theta3) * pow(sin(theta2),2) * sin(theta3) / 0.4e1 + M3 * pow(thetad1,2)* pow(L3,2)* pow(sin(theta1),2) * pow(sin(theta3),2) * cos(theta2) * sin(theta2) / 0.4e1 - M3 * pow(thetad1,2)* pow(L3,2)* pow(sin(theta1),2) * pow(sin(theta3),2) * sin(theta2) * cos(theta2) / 0.4e1 - M3 * pow(thetad1,2)* pow(L3,2)* pow(sin(theta1),2) * sin(theta3) * pow(sin(theta2),2) * cos(theta3) / 0.4e1;
      V12 = 0;
      V13 = 0;
      V22 = 0;
      V23 = 0;
      V32 = 0;
      V33 = 0;

      //Gravity Vector
      G1 = 0;
      //G2 = ((1/2)*M2*L2*cos(theta2)+M3*((1/2)*L3*cos(theta3+theta2)+L2*cos(theta2)))*-g;
      //G3 = (1/2)*M3*-g*L3*cos(theta3+theta2);
      G2 = (0.03640513331*cos(theta2)+0.01605615830*cos(theta3+theta2))*(-9.801);
      G3 = 0.01605615830*cos(theta3+theta2)*(-9.801);

      //Serial.println(G1/0.00336);
      //Serial.println(G2/0.00336);
      //Serial.println(G3/0.00336);
      // Torques
      tau[0] = H11*thetadd1 + H12*thetadd2 + H13*thetadd3 + V11*thetad1 + V12*thetad2 + V13*thetad3 + G1;
      tau[1] = H21*thetadd1 + H22*thetadd2 + H23*thetadd3 + V21*thetad1 + V22*thetad2 + V23*thetad3 + G2;
      tau[2] = H31*thetadd1 + H32*thetadd2 + H33*thetadd3 + V31*thetad1 + V32*thetad2 + V33*thetad3 + G3;

  }
  //Private variables
  double current_Position[3], current_Velocity[3];
  double current_Degree[3], current_Degs[3];
  double servo_pos_goal_one, servo_pos_goal_two, servo_pos_goal_three;
  int tra_time;
  int last_exercise;
  double servo_one_goal, servo_two_goal, servo_three_goal;
  double servo_one_a0, servo_one_a1, servo_one_a2, servo_one_a3;
  double servo_two_a0, servo_two_a1, servo_two_a2, servo_two_a3;
  double servo_three_a0, servo_three_a1, servo_three_a2, servo_three_a3;
  //Dynamic model variables 
  //Setting up link propperties
  double L2,L3,M2,M3;
  //Inertia tensors
  double I1xx,I1yx,I1zx,I1xy,I1yy,I1zy,I1xz,I1yz,I1zz;
  double I2xx,I2yx,I2zx,I2xy,I2yy,I2zy,I2xz,I2yz,I2zz;
  double I3xx,I3yx,I3zx,I3xy,I3yy,I3zy,I3xz,I3yz,I3zz;
  //Simulation propperties
  double t,var;
  double H11,H12,H13,H21,H22,H23,H31,H32,H33,V11,V12,V13,V21,V22,V23,V31,V32,V33,G1,G2,G3,tau_1,tau_2,tau_3;
}; //class Trajectory

Trajectory trajectory; //Creating object from the Trajectory class
//***** Global functions *****
void setProfVel(int ID){
  Dynamixel.setProfileVelocity(0x01, ID);  //Set the Profile Velocity for each servo. (max. is 1023)
  Dynamixel.setProfileVelocity(0x02, ID);  //Set the Profile Velocity for each servo. (max. is 1023)
  Dynamixel.setProfileVelocity(0x03, ID);  //Set the Profile Velocity for each servo. (max. is 1023)
}

void setProfAcc(int ID){
  Dynamixel.setProfileAcceleration(0x01, ID);  //Set the Profile Acceleration for each servo. (max. is 32767)
  Dynamixel.setProfileAcceleration(0x02, ID);  //Set the Profile Acceleration for each servo. (max. is 32767)
  Dynamixel.setProfileAcceleration(0x03, ID);  //Set the Profile Acceleration for each servo. (max. is 32767)
}

//Callback for the gripper subscriber
void gripCallback(const std_msgs::UInt8& grip){
  // open the gripper
  if(!trajectory.gripper_activated && wait_on_gripper){
    if(grip.data == 1){
      Dynamixel.setGoalPosition(4, 3067);
      Dynamixel.setGoalPosition(5, 1022);
      trajectory.gripper_activated = true;
    }
  }
  // close the gripper
  if(trajectory.gripper_activated && wait_on_gripper){
    if(grip.data == 0){
      wait_on_gripper = false;
      Dynamixel.setGoalPosition(4, 2045);
      Dynamixel.setGoalPosition(5, 2045);
    }
  }
}// void gripCallback

//Callback for the exercise subscriber
void exerciseCallback(const std_msgs::UInt8& Exercise){
  if (!exercise_done){
    trajectory.current_exercise = Exercise.data;
    exercise_done = true;
    trajectory.setExercise();
  }
}//Void exerciseCallback

//Callback for the trajectory time subscriber
void timeCallback(const std_msgs::UInt8& timemsg){
  if (!done){
    temp_time +=timemsg.data;
    counter++;
  }
  if (counter == 200){
    trajectory_time = temp_time/counter;
    counter = 0;
    temp_time =0;
    done = true;
  }
}//Void timeCallback
//Setting up nodehandle
ros::NodeHandle nh;
//Setting up the subscribers and hooking them up to their respective topics and callbacks
ros::Subscriber<std_msgs::UInt8> grip_sub("/Gripper", gripCallback);
ros::Subscriber<std_msgs::UInt8> time_sub("/Trajectory_time", timeCallback);
ros::Subscriber<std_msgs::UInt8> exercise_sub("/Exercise", exerciseCallback);

void beginSerial(){
  Serial.flush();                                       // Clear the serial buffer of garbage data before running the code.
  mySerial.begin(SERVO_SET_Baudrate);                   // We now need to set Ardiuno to the new Baudrate speed 57600
  Dynamixel.begin(mySerial);                            // Using mySerial object which has pin 10 as the RX serial pin, and pin 11 as the TX serial pin
  Dynamixel.setDirectionPin(SERVO_ControlPin);
  nh.initNode(); //Setting up the ROS nodehandle
  nh.subscribe(grip_sub);//Subscribing to the gripper topic
  nh.subscribe(time_sub);//Subscribing to the trajectorytime topic
  nh.subscribe(exercise_sub);//Subscribing to the exercise topic
  setTorque(true);			// Turn on holding torque.
}//Void beginSerial

/*______________Setup______________*/
void setup(){
  Serial.begin(57600);     // Start serial communication on baudrate 57600
  beginSerial();//Start the serial communication
  setProfVel(25);//Set profile velocity for internal controller trajectory
  setProfAcc(10);//Set profile acceleration for internal controller trajectory
  for(int i=0; i<3; i++){
    Dynamixel.setGoalCurrent(i+1, 3/0.00336);//Setting the torque for servo 1-3
  }
}//Void setup

/*______________Loop______________*/


void loop(){
  trajectory.posReached();
  if (!trajectory.prot_stop){
    trajectory.reachableWorkspace();
    if(!trajectoryDone){
      if(run_time == 0){
      trajectory.dynamic_boost = false;
      trajectory.setExercise();
      trajectory.planTra(trajectory_time);
      }
      trajectory.calcTra();
    }
  }
}// void loop()
