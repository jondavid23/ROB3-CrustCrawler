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
int global_exercise;
class EMG_subpub
{
public:

  EMG_subpub()
  {
    //Topics to publish
    vel_pub = nh.advertise<std_msgs::UInt8>("Velocity", 1);
    exercise_pub = nh.advertise<std_msgs::UInt8>("Exercise", 1);
    gripper_pub = nh.advertise<std_msgs::UInt8>("Gripper",1);
    //Topic to subscribe from
    raw_sub = nh.subscribe("/myo_raw/myo_emg", 1, &EMG_subpub::Raw_Callback,this);
    gest_sub = nh.subscribe("/myo_raw/myo_gest_str",1,&EMG_subpub::Gest_Callback,this);
  }
  // Function to get the average of the EMG array
  int avgArr(int arr[8])
  {
    int n,sum,avg;
    sum = arr[0]+arr[1]+arr[2]+arr[3]+arr[4]+arr[5]+arr[6]+arr[7];
    n = 8;
    avg = sum/n;
    return avg;
  }
  // Callback function to get gesture data and publish open or closed position of the gripper to CrustCrawler
  void Gest_Callback(const std_msgs::String str)
  {
    std_msgs::UInt8 gripper_msg;
    if (str.data == gripper_open)
    {
      gripper = 1;
    }
    else if (str.data == gripper_close)
    {
      gripper = 0;
    }
    gripper_msg.data = gripper;
    gripper_pub.publish(gripper_msg);
  }
  // Gets the raw EMG data and sets velocity threshold for Trajectory planning
  void Raw_Callback(const ros_myo::EmgArray msg)
  {
    exercise = global_exercise;
    std_msgs::UInt8 vel_msg;
    std_msgs::UInt8 exercise_msg;
    for(int i=0; i<8; i++)
    {
      arrEmg[i] = msg.data[i];
      // For Debugging
      //std::cout << msg.data[i]<< " "  ;
    }
    //std::cout << "]" << std::endl;
    threshold = avgArr(arrEmg);
    //std::cout << threshold << std::endl;
    if (threshold < 50)
    {
      velocity = 0;
    }
    else if (threshold < 100)
    {
      velocity = 9;
    }
    else if (threshold < 500)
    {
      velocity = 6;
    }
    else if (500 < threshold)
    {
      velocity = 3;
    }
    vel_msg.data = velocity;
    exercise_msg.data = exercise;
    vel_pub.publish(vel_msg);
    exercise_pub.publish(exercise_msg);
  }

private:
  ros::NodeHandle nh;
  ros::Publisher vel_pub;
  ros::Publisher exercise_pub;
  ros::Publisher gripper_pub;
  ros::Subscriber raw_sub;
  ros::Subscriber gest_sub;
  int threshold,velocity,exercise,gripper;
  int arrEmg[8];
  std::string gripper_open = "WAVE_OUT";
  std::string gripper_close ="WAVE_IN";
};//End of class EMG_subpub
// Function that sellects what exercise to perform on the CrustCrawler
void Select_exercise ()
{
  int usr_input;
  std::cout << "***************************"<< std::endl;
  std::cout << "Please select an Exercise." << std::endl;
  std::cout << "Available Exercises are: " << std::endl;
  std::cout << "Exercise 1, Exercie 2, Exercise 3." << std::endl;
  std::cout << "Please press 1, 2 or 3 and then press 'Enter' to select the exercise" << std::endl;
  std::cin >> usr_input;
  std::cout << "Exercise "<<usr_input<<" selected." << std::endl;
  std::cout << "***************************"<< std::endl;
  global_exercise = usr_input;
}
int main(int argc, char **argv)
{
  //Select exercise to perform on CrustCrawler
  Select_exercise();
  //Initiate ROS
  ros::init(argc, argv, "EMG_processing");
  //Create an object of class SubscribeAndPublish that will take care of everything
  EMG_subpub emg;
  ros::spin();
  return 0;
}
