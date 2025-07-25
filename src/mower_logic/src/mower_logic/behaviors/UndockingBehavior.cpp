// Created by Clemens Elflein on 2/21/22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
#include "UndockingBehavior.h"
#include <tf2_ros/transform_listener.h>

extern ros::ServiceClient dockingPointClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfClientExePath;
extern xbot_msgs::AbsolutePose getPose();
extern mower_msgs::Status getStatus();
extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *mbfClient;
extern tf2_ros::Buffer tfBuffer;

extern void setRobotPoseDocked();
extern void stopMoving();
extern bool isGpsGood();
extern bool setGPS(bool enabled);
extern bool setGPSRtkFloat(bool enabled);

UndockingBehavior UndockingBehavior::INSTANCE(&MowingBehavior::INSTANCE);
UndockingBehavior UndockingBehavior::RETRY_INSTANCE(&DockingBehavior::INSTANCE);

std::string UndockingBehavior::state_name() {
    return "UNDOCKING";
}

Behavior *UndockingBehavior::execute() {
    // set the robot's position to the dock if we're actually docked
    if(getStatus().v_charge > 5.0) {
        ROS_INFO_STREAM("Currently inside the docking station, we set the robot's pose to the docks pose.");

        setRobotPoseDocked();
    }
    // wait for the filters
    ros::Duration(1.0).sleep();

    ROS_INFO("Undocking: getting initial pose from tf");
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    try {
        geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
        ROS_INFO_STREAM("om_mower_logic map/base_link: " << transformStamped.transform.translation << " - " << transformStamped.transform.rotation);
        pose.pose.position.x = transformStamped.transform.translation.x;
        pose.pose.position.y = transformStamped.transform.translation.y;
        pose.pose.orientation.x = transformStamped.transform.rotation.x;
        pose.pose.orientation.y = transformStamped.transform.rotation.y;
        pose.pose.orientation.z = transformStamped.transform.rotation.z;
        pose.pose.orientation.w = transformStamped.transform.rotation.w;
    }
    catch (tf2::TransformException &ex) {
        ROS_ERROR("om_mower_logic map/base_link: %s", ex.what());
        return &IdleBehavior::INSTANCE;
    }

    // get robot's current pose from odometry.
    // xbot_msgs::AbsolutePose pose = getPose();
    tf2::Quaternion quat;
    tf2::fromMsg(pose.pose.orientation, quat);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    mbf_msgs::ExePathGoal exePathGoal;

    nav_msgs::Path path;


    int undock_point_count = config.undock_distance * 10.0;
    // for (int i = 0; i < undock_point_count; i++) { // point 0 is the dock so no use to put it here
    for (int i = 1; i < undock_point_count; i++) {
        geometry_msgs::PoseStamped docking_pose_stamped_front;
        docking_pose_stamped_front.pose = pose.pose;
        docking_pose_stamped_front.header = pose.header;
        docking_pose_stamped_front.pose.position.x -= cos(yaw) * (i / 10.0);
        docking_pose_stamped_front.pose.position.y -= sin(yaw) * (i / 10.0);
        path.poses.push_back(docking_pose_stamped_front);
        // ROS_INFO("Undocking point %d: %f %f", i, docking_pose_stamped_front.pose.position.x,
        //          docking_pose_stamped_front.pose.position.y);
    }

    exePathGoal.path = path;
    exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
    exePathGoal.dist_tolerance = 0.1;
    exePathGoal.tolerance_from_action = true;
    exePathGoal.controller = "DockingFTCPlanner";

    actionlib::SimpleActionClient<mbf_msgs::ExePathAction> undockMbfClientExePath("/move_base_flex/exe_path");
    auto result = undockMbfClientExePath.sendGoalAndWait(exePathGoal, ros::Duration(60.0), ros::Duration(60.0));

    bool success = result.state_ == actionlib::SimpleClientGoalState::SUCCEEDED;

    if (!success) {
        ROS_ERROR_STREAM("Error during undock");
        return &IdleBehavior::INSTANCE;
    }

    // Goto the fix point
    if (config.gps_use_fix_point) {
        ROS_INFO_STREAM("Reaching fix point");
        // allow it no navigate with float rtk
        setGPSRtkFloat(true);
        bool hasGps = waitForGPS();
        if (!hasGps) {
            ROS_ERROR_STREAM("Could not get GPS.");
            return &IdleBehavior::INSTANCE;
        }

        geometry_msgs::PoseStamped fix_point;
        fix_point.header.frame_id = "map";
        fix_point.pose.position.x = config.gps_fix_point_x;
        fix_point.pose.position.y = config.gps_fix_point_y;
        fix_point.pose.position.z = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, 0);
        fix_point.pose.orientation = tf2::toMsg(quat);
        mbf_msgs::MoveBaseGoal moveBaseGoal;
        moveBaseGoal.target_pose = fix_point;
        moveBaseGoal.controller = "FTCPlanner";
        actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> undockMbfClient("/move_base_flex/move_base");
        auto result = undockMbfClient.sendGoalAndWait(moveBaseGoal);
        if (result.state_ != result.SUCCEEDED) {
            ROS_ERROR_STREAM("Error reaching fix point");
            return &IdleBehavior::INSTANCE;
        }
        // now we want clean rtk fix
        setGPSRtkFloat(false);
    }

    // stop the bot for now
    stopMoving();

    ROS_INFO_STREAM("Undock success. Waiting for GPS.");
    bool hasGps = waitForGPS();

    if (!hasGps) {
        ROS_ERROR_STREAM("Could not get GPS.");
        return &IdleBehavior::INSTANCE;
    }

    // TODO return mow area
    return nextBehavior;

}

void UndockingBehavior::enter() {
    reset();
    paused = aborted = false;
}

void UndockingBehavior::exit() {

}

void UndockingBehavior::reset() {
    gpsRequired = false;
}

bool UndockingBehavior::needs_gps() {
    return gpsRequired;
}

bool UndockingBehavior::mower_enabled() {
    // No mower during docking
    return false;
}

bool UndockingBehavior::waitForGPS() {
    gpsRequired = false;
    setGPS(true);
    ros::Rate odom_rate(1.0);

    // wait at least config.gps_wait_time for gps rtk fix. it must be fixed during all the period
    auto start = ros::Time::now();
    while (start + ros::Duration(config.gps_wait_time, 0) > ros::Time::now()) {
        if (!ros::ok() || aborted) {
            return false;
        }
        if (!isGpsGood()) {
            start = ros::Time::now();
        }
        odom_rate.sleep();
    }

    gpsRequired = true;

    return true;
}

UndockingBehavior::UndockingBehavior(Behavior* next) {
    this->nextBehavior = next;
}

void UndockingBehavior::command_home() {

}

void UndockingBehavior::command_start() {

}

void UndockingBehavior::command_s1() {

}

void UndockingBehavior::command_s2() {

}

bool UndockingBehavior::redirect_joystick() {
    return false;
}


uint8_t UndockingBehavior::get_sub_state() {
    return 2;

}
uint8_t UndockingBehavior::get_state() {
    return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

void UndockingBehavior::handle_action(std::string action) {
}
