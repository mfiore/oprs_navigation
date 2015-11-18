/*
 * ControlSpeed.cpp
 *
 *  Created on: Jun 7, 2015
 *      Author: mfiore
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
#include "geometry_msgs/Twist.h"
#include "spencer_control_msgs/SetMaxVelocity.h"
#include <algorithm>    // std::min


using namespace std;


ros::ServiceClient controlSpeed;

double actualSpeed=0.9;
double minSpeed=0.5;
double maxSpeed=1.1;


static string dest="OPRS_SUP";
int mpSocket;
string mpHost;

string getNext(char *message, int *i) {
    string ret;
    while (message[*i]!=' ' && message[*i]!=')'){
        ret=ret+message[*i];
        (*i)++;
    }
    (*i)++;
    return ret;
}


vector<double> getVelocity(string command) {
	vector<double> result;
	if (command=="accelerate") {
		double newSpeed=min(maxSpeed,actualSpeed+0.1);
		actualSpeed=newSpeed;
		result.push_back(newSpeed);
		result.push_back(0.3);
	}
	else if (command=="decelerate") {
		double newSpeed=max(minSpeed,actualSpeed-0.1);
		actualSpeed=newSpeed;
		result.push_back(newSpeed);
		result.push_back(0.3);
	}
	return result;

}
void oprsLoop() {
	int length;
	while (ros::ok()) {
		char * sender=read_string_from_socket(mpSocket,&length);
		char * message=read_string_from_socket(mpSocket,&length);

		cout<<"Received message "<<message<<"\n";

		int i=0;
		getNext(message,&i);
		string command=getNext(message,&i);

		spencer_control_msgs::SetMaxVelocity srv;

		if (command=="accelerate" || command=="decelerate") {
			vector<double> velocity=getVelocity(command);
			srv.request.max_linear_velocity=velocity[0];
			srv.request.max_angular_velocity=0.4;
			controlSpeed.call(srv);
			cout<<"new speed is "<<velocity[0]<<" "<<velocity[1]<<"\n";
		}
		string strMessage=("(control_speed.report OK)");
		char returnMessage[50];
		strcpy(returnMessage,strMessage.c_str());
		send_message_string(returnMessage,dest.c_str());


	}

}


int main(int argc, char ** argv) {
	if (argc<2) {
		cout<<"usage hostName\n";
		return 0;
	}
	mpHost=argv[1];

	ros::init(argc,argv,"control_speed");
	ros::NodeHandle n;
	cout<<"Started control_speed\n";

	controlSpeed=n.serviceClient<spencer_control_msgs::SetMaxVelocity>("/spencer/control/set_max_velocity",true);
	cout<<"Connected to the service\n";

	spencer_control_msgs::SetMaxVelocity srv;
	srv.request.max_linear_velocity=actualSpeed;
	srv.request.max_angular_velocity=0.4;
	controlSpeed.call(srv);
	cout<<"Starting speed is "<<actualSpeed<<"\n";

	mpSocket=external_register_to_the_mp_host_prot("control_speed",mpHost.c_str(),3300,STRINGS_PT);

	if (mpSocket!=-1) {
		cout<<"Connected to message parser on "<<mpHost<<" "<<"control_speed\n";
		boost::thread t(oprsLoop);
		ros::waitForShutdown();
	}


}
