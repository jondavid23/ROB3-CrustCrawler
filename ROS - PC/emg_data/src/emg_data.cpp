//****************************************************************//
/*This file connects to the MyoArmband and gets the raw EMG data
  from the myo along with certain gestures. This data is proccesed
  and published over the ros framework to be used by the CrustCrawler
  Autor: Oliver Emil Trudslev Hansen
  email: oeth16@student.aau.dk
*/
//***************************************************************//
#include <iostream>
#include "ros/ros.h"
#include <ros_myo/EmgArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt8.h>
#include <string>
int global_exercise,time_a,time_b,time_c,time_d;
class EMGSubpub{
public:

  EMGSubpub(){
    //Topics to publish
    time_pub = nh.advertise<std_msgs::UInt8>("Trajectory_time", 1);
    exercise_pub = nh.advertise<std_msgs::UInt8>("Exercise", 1);
    gripper_pub = nh.advertise<std_msgs::UInt8>("Gripper",1);
    //Topic to subscribe from
    raw_sub = nh.subscribe("/myo_raw/myo_emg", 1, &EMGSubpub::rawCallback,this);
    gest_sub = nh.subscribe("/myo_raw/myo_gest_str",1,&EMGSubpub::gestCallback,this);
  }//EMGSubpub Constructor
  // Function to get the average of the EMG array
  int avgArr(int arr[8]){
    int n,sum,avg;
    sum = arr[0]+arr[1]+arr[2]+arr[3]+arr[4]+arr[5]+arr[6]+arr[7];
    n = 8;
    avg = sum/n;
    return avg;
  } //Int avgArr

  // Callback function to get gesture data and publish open or closed position of the gripper to CrustCrawler
  void gestCallback(const std_msgs::String str){
    std_msgs::UInt8 gripper_msg;
    if (str.data == gripper_open){
      gripper = 1;
    }
    else if (str.data == gripper_close){
      gripper = 0;
    }
    gripper_msg.data = gripper;
    gripper_pub.publish(gripper_msg);
  } //Void gestCallback

  // Gets the raw EMG data and sets velocity threshold for Trajectory planning
  int i = 0;
  bool done;
  void rawCallback(const ros_myo::EmgArray msg){
    exercise = global_exercise;
    std_msgs::UInt8 time_msg;
    std_msgs::UInt8 exercise_msg;
    for(int i=0; i<8; i++){
      arrEmg[i] = msg.data[i];
    }
    threshold = avgArr(arrEmg);
    if (threshold>0 && threshold <= 100){
      //time_a = 16;
      trajectory_time = time_a;
    }
    else if (threshold>100 && threshold <= 300){
      //time_b = 12;
      trajectory_time = time_b;
    }
    else if (threshold>300 && threshold <= 500){
      //time_c = 8;
      trajectory_time = time_c;
    }
    else if (threshold>500){
      //time_d = 4;
      trajectory_time = time_d;
    }
    time_msg.data =trajectory_time;
    exercise_msg.data = exercise;
    time_pub.publish(time_msg);
    exercise_pub.publish(exercise_msg);
  } //Void rawCallback

private:
  ros::NodeHandle nh;
  ros::Publisher time_pub;
  ros::Publisher exercise_pub;
  ros::Publisher gripper_pub;
  ros::Subscriber raw_sub;
  ros::Subscriber gest_sub;
  int threshold,trajectory_time,exercise,gripper;
  int arrEmg[8];
  std::string gripper_open = "WAVE_OUT";
  std::string gripper_close ="WAVE_IN";
}; //End of class EMG_subpub

// Function that sellects what exercise to perform on the CrustCrawler
void selectExercise()
{
  int usr_input;
  std::cout << "***********************************************************************"<< std::endl;
  std::cout << "Please select an Exercise." << std::endl;
  std::cout << "Available Exercises are: " << std::endl;
  std::cout << "Exercise 1, Exercie 2, Exercise 3." << std::endl;
  std::cout << "Please press 1, 2 or 3 and then press 'Enter' to select the exercise" << std::endl;
  std::cin >> usr_input;
  std::cout << "Exercise "<<usr_input<<" selected." << std::endl;
  std::cout << "***********************************************************************"<< std::endl;
  global_exercise = usr_input;
  std::cout << "Please select the trajectory time settings for the MyoArmband."<< std::endl;
  std::cout << "To set the trajectory times press a number and then hit enter."<< std::endl;
  std::cout << "This must be done 4 times."<< std::endl;
  std::cout << "Input trajectory time value 1"<< std::endl;
  std::cin >>time_a;
  std::cout << "Input trajectory time value 2"<< std::endl;
  std::cin >>time_b;
  std::cout << "Input trajectory time value 3"<< std::endl;
  std::cin >>time_c;
  std::cout << "Input trajectory time value 4"<< std::endl;
  std::cin >>time_d;
  std::cout << "The trajectory times have now been set."<< std::endl;
  std::cout << "***********************************************************************"<< std::endl;
} //Void selectExercise
int main(int argc, char **argv)
{
  //Select exercise to perform on CrustCrawler
  selectExercise();
  //Initiate ROS
  ros::init(argc, argv, "EMG_processing");
  //Create an object of class SubscribeAndPublish that will take care of everything
  EMGSubpub emgsubpub;
  ros::spin();
  return 0;
} //Int main
