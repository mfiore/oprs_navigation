/**
   This node acts as an interface between the OPRS supervision system and the navigation ros nodes.
   It receives messages in this format (for now) and then contacts /move/base , updating the supervisor with a 
   BaseActions.update  and BaseActions.response messages.

   (move @mode @coordinates)
   (stop)
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
#include <boost/thread/thread.hpp>

using namespace std;


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
static string dest="OPRS_SUP";

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

vector<geometry_msgs::PoseStamped> getMoveMessage(char * message, int i) {
    //we start getting the mode of the message
  
  
  vector<geometry_msgs::PoseStamped>  posesStamped;

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

	return posesStamped;
    }
}






void moveRobot(vector<geometry_msgs::PoseStamped> posesStamped, Client *client) {


    ROS_INFO("ready to navigate");
    //nowe we are actually going to send this stuff to move_base and update the supervisor step by step     
    int n=posesStamped.size();
    string response="OK";
    
    for (int i=0; i<n && response!="FAILURE"; i++) {
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose=posesStamped[i];
	client->sendGoal(goal);
	ROS_INFO("goal sent %f %f %f %f %f %f",goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.x , goal.target_pose.pose.orientation.y , goal.target_pose.pose.orientation.z , goal.target_pose.pose.orientation.w);
	ROS_INFO("goal sent %f %f",posesStamped[i].pose.position.x, posesStamped[i].pose.position.y);
	client->waitForResult(ros::Duration(5.0));
	ROS_INFO("received response %s", client->getState().toString().c_str());
	while (client->getState() == actionlib::SimpleClientGoalState::ACTIVE) {

	    client->waitForResult(ros::Duration(5.0));
	}
	ROS_INFO("%s", client->getState().toString().c_str());
	if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED )
	    response="OK";
	else {
	    response="FAILURE";
	}
	//we update the supervisor with the current position
	printf("Current State: %s\n", client->getState().toString().c_str());
	string strmessage="(Navigation.move.update "+response+")";
	char returnMessage[50];
	strcpy(returnMessage,strmessage.c_str());
	ROS_INFO("Return message %s",returnMessage);
	send_message_string(returnMessage,dest.c_str());
    }
    //and when we have finished give a final update 
    string strmessage="(Navigation.move.report "+response+")";
    char returnMessage[50];
    strcpy(returnMessage,strmessage.c_str());
    ROS_INFO("Return message %s",returnMessage);
    send_message_string(returnMessage,dest.c_str());
    
}





int main(int argc, char **argv) {
    //SET ROS 
    ros::init(argc, argv, "Navigation");

Client client("move_base",true);
    ROS_INFO("starting navigation");  
  
    //Set move_base

    client.waitForServer();
    ROS_INFO("connected to move_base");

    boost::thread *moveRobotThread;
    string status="not moving";

    //connect to openprs
    int mpSocket=external_register_to_the_mp_prot("Navigation", 3300, STRINGS_PT);
    if (mpSocket!=-1) {

	while (ros::ok()) {

	    ROS_INFO("connected to the message parser");
      
	    //read the openprs message
	    int length;
	    char *sender=read_string_from_socket(mpSocket,&length);
	    char *message=read_string_from_socket(mpSocket,&length);
      
	    ROS_INFO("message %s",message);
	    int i=0;   
	    //first we get the command of the robot
	    string command="";
	    i++;
	    while (message[i]!=' ' && message[i]!=')') {
	      command+=message[i];
		i++;   
	    }

	    std::cout<<"command "<<command<<"\n";
	    if (command=="stop") {
		client.cancelGoal();
		status="not moving";
    string strmessage="(Navigation.stop.report OK)";
    char returnMessage[50];
    strcpy(returnMessage,strmessage.c_str());
    ROS_INFO("Return message %s",returnMessage);
    send_message_string(returnMessage,dest.c_str());
	    }
	    else if (status!="moving") {
		
		vector<geometry_msgs::PoseStamped> posesStamped=getMoveMessage(message,i);
		moveRobotThread=new boost::thread(moveRobot,posesStamped, &client);
	    }
	}
    }
    else {
      ROS_INFO("error in connecting to the message passer. Closing the node");
      return 0;
    }
}

