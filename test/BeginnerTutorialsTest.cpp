/**
 * @file BeginnerTutorialsTest.cpp
 * @author Diane Ngo (dngo13)
 * @brief 
 * @version 0.1
 * @date 2021-11-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/ChangeStringOutput.h"

/**
 * @brief Tests if the service for the change string output exists
 * @param TalkerNode gtest suite
 * @param testTalkerExists name of test
 */
TEST(TalkerNode, testTalkerExists) {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<beginner_tutorials::ChangeStringOutput>("change_string");
    EXPECT_TRUE(client.waitForExistence(ros::Duration(10)));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle n;
    return RUN_ALL_TESTS();
}