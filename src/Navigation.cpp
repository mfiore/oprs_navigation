/**
   This node acts as an interface between the OPRS supervision system and the navigation ros nodes.
   It receives messages in this format (for now) and then contacts /move/base , updating the supervisor with a 
   BaseActions.update  and BaseActions.response messages.

   (move @mode @coordinates)
   @mode =  euler or quaternion
   
   @coordinates (. (. x1 y1 z1 tx1 ty1 tz1 w1 .) (. x2 y2 z2 w1 tx2 ty2 tz2 w2 .) ... (. xn yn zn txn tym tzn wn .) .)  or
                (. (. x1 y1 t1 .) (. x2 y2 t2 .) ... (. xn yn tn .))
   

   @author Michelangelo Fiore
*/

#include "ros/ros.h"
#include <string>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <vector>
#include <string>
#include <boost/lexical_cast.hpp>
#include "opaque-pub.h"
#include "mp-pub.h"
#include "geometry_msgs/PoseStamped.h";
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>

using namespace std;


/**
   Get the next number in the message, skipping spaces. Takes as input the message and the current position, returning a float with the number and updating the position included in i accordingly.   
 */
float getNextNumber(char *message, int *i) {
  int j=*i;
  string number;
  while (message[j]!=' ') {
    number=number+message[j];
    j++;
  }
  cout<<number<<"\n";
  *i=j;
  return boost::lexical_cast<float>(number);

}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

int main(int argc, char **argv) {
  string dest="OPRS_SUP";

  //SET ROS 
  ros::init(argc, argv, "Navigation");
  ROS_INFO("starting navigation");  
  
  //Set move_base
  Client client("move_base",true);
  client.waitForServer();
  ROS_INFO("connected to move_base");

  //connect to openprs
  int mpSocket=external_register_to_the_mp_prot("Navigation", 3300, STRINGS_PT);
  if (mpSocket!=-1) {
    ROS_INFO("connected to the message parser");
      
    //read the openprs message
    int length;
    char *sender=read_string_from_socket(mpSocket,&length);
    char *message=read_string_from_socket(mpSocket,&length);
      
    vector<geometry_msgs::PoseStamped> posesStamped;  //we will put all the waypoints here. For now we use move_base so maybe it's not necessary but we may switch to the laas waypoints node in the future, and that one takes a vector as input.

    ROS_INFO("message %s",message);

    //some weird and crappy string operations to get to the right lisp-list position. The message is something like (move (. (. position orientation .) .)) so we start by arriving after the first '.' 

    int i=0;
    //we start getting the mode of the message
    while (message[i]==' ') {
      i++;   
    }
    string mode;
    while (message[i]!=' ') { 
	mode=mode+message[i];  
        i++;
    }


    while(message[i]!='.'){  
      i++;                    
    }
    i++;
    
    while (message[i]!=')' && message[i]!='.') {  //we finish when we found two ')' in a row

      while (message[i]!='(') { //a '(' indicates the start of a first waypoint. 
	i++;
      }
      //we get after the first '(' and then skip after the '.' until the first number.
      i++;
      i++;
      i++;
      geometry_msgs::PoseStamped aPoseStamped;

      //now we can actually fill the message =) . Code looks less crappy here YAI!
      aPoseStamped.header.stamp=ros::Time::now();
      aPoseStamped.header.frame_id="map";
	
      aPoseStamped.pose.position.x=getNextNumber(message,&i);
      i++;
      aPoseStamped.pose.position.y=getNextNumber(message,&i);
      i++;

      if (mode=="quaternion") {
      aPoseStamped.pose.position.z=getNextNumber(message,&i);
      i++;
      

      aPoseStamped.pose.orientation.x=getNextNumber(message,&i);
      i++;
      aPoseStamped.pose.orientation.y=getNextNumber(message,&i);
      i++;
      aPoseStamped.pose.orientation.z=getNextNumber(message,&i);
      i++;
      aPoseStamped.pose.orientation.w=getNextNumber(message,&i);
      i++;
      } else {
        aPoseStamped.pose.position.z=0.0;

	double angle= getNextNumber(message,&i);
	aPoseStamped.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,angle);
        i++;
      }


      //and here is crappy again! Yai! We skip to after the point of '.)' or something like that. Maybe something like that.
      i++;
      i++;
      i++;
      //  i++;
      ROS_INFO("%d",i);
      ROS_INFO("%c", message[i]);

      posesStamped.push_back(aPoseStamped);
    }

    ROS_INFO("ready to navigate");
    //nowe we are actually going to send this stuff to move_base and update the supervisor step by step     
    int n=posesStamped.size();
    string response="OK";
    
    for (int i=0; i<n && response!="FAILURE"; i++) {
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose=posesStamped[i];
      client.sendGoal(goal);
      ROS_INFO("goal sent %f %f",goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
      ROS_INFO("goal sent %f %f",posesStamped[i].pose.position.x, posesStamped[i].pose.position.y);
      client.waitForResult(ros::Duration(5.0));
      ROS_INFO("received response %s", client.getState().toString().c_str());
      while (client.getState() == actionlib::SimpleClientGoalState::ACTIVE) {

	client.waitForResult(ros::Duration(5.0));
      }
      ROS_INFO("%s", client.getState().toString().c_str());
      if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED )
	response="OK";
      else {
	response="FAILURE";
      }
      //we update the supervisor with the current position
      printf("Current State: %s\n", client.getState().toString().c_str());
      string strmessage="(Navigation.update "+response+")";
      char returnMessage[50];
      strcpy(returnMessage,strmessage.c_str());
      ROS_INFO("Return message %s",returnMessage);
      send_message_string(returnMessage,dest.c_str());
    }
    //and when we have finished give a final update 
    string strmessage="(Navigation.report "+response+")";
    char returnMessage[50];
    strcpy(returnMessage,strmessage.c_str());
    ROS_INFO("Return message %s",returnMessage);
    send_message_string(returnMessage,dest.c_str());
    
  }
  else {
    ROS_INFO("error in connecting to the message passer. Closing the node");
    return 0;
  }
}

