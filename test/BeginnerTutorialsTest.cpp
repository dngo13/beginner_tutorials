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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "beginner_tutorials/ChangeStringOutput.h"

TEST(TalkerNode, testTalkerExists) {
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<beginner_tutorials::ChangeStringOutput>("change_string");

    bool exist(client.waitForExistence(ros::Duration(10)));
    EXPECT_FALSE(exist);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle n;
    return RUN_ALL_TESTS();
}