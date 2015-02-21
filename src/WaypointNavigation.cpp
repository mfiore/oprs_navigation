/**
   This node acts as an interface between the OPRS supervision system and the navigation ros nodes.
   It receives messages from ops, reads a trajectory poster and contacts the waypoint node to move the robot.

   (move)
   (stop)

   @author Benjamin Vadant, Michelangelo Fiore
*/

#include <portLib.h>
#include <posterLib.h>
#include "genom/genomError.h"

#include "opaque-pub.h"
#include "mp-pub.h"
#include "mhpStructBen.h"
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <waypoints/waypointsAction.h>

#include <string>
#include <iostream>
#include <boost/thread/thread.hpp>

using namespace std;

typedef actionlib::SimpleActionClient<waypoints::waypointsAction> Client;
static string dest="OPRS_SUP";


void moveRobot(	POSTER_ID posterI, Client *client) {
        {
                STRUCT_MHP_BEN_BASE_TRAJ* traj;

                if (posterRead(posterId, 0, (char *)traj, length) != length) {
                    string strmessage="(Navigation.response FAILURE)";
                    char returnMessage[50];
                    strcpy(returnMessage,strmessage.c_str());
                    ROS_INFO("Return message %s",returnMessage);
                    send_message_string(returnMessage,dest.c_str());
                    return ERROR;
                }

                waypoints::waypointsGoal goal = WayPoints::convert(traj);

                ac.sendGoal(goal);

                ROS_INFO("Goal sended");

                //wait for the action to return
                bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

                string response;
                if (finished_before_timeout)
                {
                    actionlib::SimpleClientGoalState state = ac.getState();
                    ROS_INFO("Action finished: %s",state.toString().c_str());
                    response="OK";

                }
                else {
                ROS_INFO("Action did not finish before the time out.");
                response="FAILURE";
                }
                string strmessage="(Navigation.response "+response+")";
                char returnMessage[50];
                strcpy(returnMessage,strmessage.c_str());
                ROS_INFO("Return message %s",returnMessage);
                send_message_string(returnMessage,dest.c_str());
        }





int main(int argc, char **argv) {
    //SET ROS
    ros::init(argc, argv, "Navigation");

    Client client("waypoints",true);
    ROS_INFO("starting WaypointNavigation");

    //Set move_base
    client.waitForServer();
    ROS_INFO("connected to waypoints");

    boost::thread *moveRobotThread;

    POSTER_ID posterId;
    size_t length;
    if (posterFind("BaseTraj", posterId) == ERROR) {
            *posterId=NULL;
            return ERROR;
    }
    if (posterIoctl(*posterId, FIO_GETSIZE, &length) == ERROR) {
            *posterId=NULL;
            return ERROR;
    }
    if (length != sizeof(STRUCT_MHP_BEN_BASE_TRAJ)) {
            return ERROR;
    }


    //connect to openprs
    int mpSocket=external_register_to_the_mp_prot("WaypointNavigation", 3300, STRINGS_PT);
    if (mpSocket!=-1) {

    while (ros::ok()) {

        ROS_INFO("connected to the message parser");

        //read the openprs message
        int length;
        char *sender=read_string_from_socket(mpSocket,&length);
        char *message=read_string_from_socket(mpSocket,&length);

        ROS_INFO("message %s",message);
        if (message=="(stop)") {
            client.cancelGoal();
            string strmessage="(WaypointNavigation.stop.report OK)";
            char returnMessage[50];
            strcpy(returnMessage,strmessage.c_str());
            ROS_INFO("Return message %s",returnMessage);
            send_message_string(returnMessage,dest.c_str());
        }
        else if(message=="(move)") {

            moveRobotThread=new boost::thread(moveRobot, posterId, &client);
         }
    }
    }
    else {
      ROS_INFO("error in connecting to the message passer. Closing the node");
      return 0;
    }
}
