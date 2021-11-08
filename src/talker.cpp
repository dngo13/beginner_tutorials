/**
 * @file talker.cpp
 * @author Diane Ngo(dngo13)
 * @brief ROS Beginner Tutorials - Publisher
 * @version 2.0
 * @date 2021-11-08
 * 
 * @copyright BSD 2 (c) 2021
 * 
 */

// ROS Headers
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/ChangeStringOutput.h"

/**
 * @brief Callback function for the service to change the string output
 * 
 * @param req Request
 * @param res Reponse
 * @return bool if there is a request to change the string
 */
bool change_string(beginner_tutorials::ChangeStringOutput::Request request,
beginner_tutorials::ChangeStringOutput::Response response) {
  if (!request.baseString.empty()) {
    Message = request.baseString;
    response.newString = "Service called to update string output!";
    ROS_WARN_STREAM_ONCE("String has been updated.");
    return true;
  } else {
    ROS_ERROR_STREAM("New string cannot be empty. Will not be updated.");
    return false; }
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  // INFO logger for node start
  ROS_INFO_STREAM_ONCE("Starting talker node. ");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // Default frequency is 10Hz
  int frequency = 10;
  // Passing argument from terminal and converting it from string to int
  if (argc == 2) {
    frequency = atoi(argv[1]);
    ROS_WARN_STREAM("Frequency set to: " << frequency);
  }
  // Checks if frequency is negative. Terminates program.
  if (frequency <= 0) {
    ROS_FATAL_STREAM_ONCE("ERROR: Frequency cannot be zero or negative.");
    ros::shutdown();
  } else if (frequency >= 25) {
  // Checks if frequency is greater than 25
    ROS_ERROR_STREAM("Frequency is too high (> 25Hz).");
  }
  ros::Rate loop_rate(frequency);

  // Default string base message
  std::string Message("ENPM808X ROS count:  ");

  // Start service server for ChangeStringOutput
  ros::ServiceServer server = n.advertiseService("ChangeStringOutput",
  change_string);
  ROS_DEBUG_STREAM("Starting ChangeStringOutput service");

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << Message << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
