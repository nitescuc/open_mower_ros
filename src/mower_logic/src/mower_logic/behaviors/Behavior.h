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
#ifndef SRC_BEHAVIOR_H
#define SRC_BEHAVIOR_H

#include "ros/ros.h"
#include "mower_logic/MowerLogicConfig.h"
#include "mower_msgs/HighLevelStatus.h"
#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <atomic>
#include <memory>
#include <functional>

enum eAutoMode {
    MANUAL = 0,
    SEMIAUTO = 1,
    AUTO = 2
};

struct sSharedState {
    bool active_semiautomatic_task;
};

/**
 * Behavior definition
 */
class Behavior {

private:
    ros::Time startTime;

protected:
    std::atomic<bool> aborted;
    std::atomic<bool> paused;

    std::atomic<bool> requested_continue_flag;
    std::atomic<bool> requested_pause_flag;
    std::atomic<bool> requested_crash_recovery_flag;

    std::atomic<bool> isGPSGood;
    std::atomic<bool> isGPSFixed;
    std::atomic<uint8_t> sub_state;

    double time_in_state() {
        return (ros::Time::now() - startTime).toSec();
    }

    mower_logic::MowerLogicConfig config;
    std::shared_ptr<sSharedState> shared_state;

    /**
     * Called during goal execution to allow derived classes to influence the execution based on state.
     * Base implementation handles abort flag, charging detection, GPS/emergency timeout, and progress monitoring.
     * 
     * @param state The current action state (SimpleClientGoalState::state_)
     * @return < 0 to treat as error (cancel + fail), > 0 to treat as success (cancel + succeed), 0 to continue normally
     */
    virtual int on_progress(int state);

    /**
     * Execute a goal using MBF (Move Base Flex) with progress monitoring and error handling.
     * This method sends a goal to the MoveBase action client and monitors its execution,
     * handling GPS loss, emergency mode, and charging detection.
     * Calls on_progress() to allow derived classes to influence execution.
     * 
     * @param client The MoveBase action client to use
     * @param goal The goal to execute
     * @return true if the goal was successfully reached or charging detected, false otherwise
     */
    bool execute_goal(
        actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *client,
        mbf_msgs::MoveBaseGoal goal
    );

    /**
     * Drive to a specific position using the default controller.
     * This is a convenience method that creates a MoveBaseGoal from a PoseStamped and executes it.
     * 
     * @param target_pose The target pose to drive to
     * @param controller The controller to use (default: "FTCPlanner")
     * @return true if the position was successfully reached, false otherwise
     */
    bool drive_to_position(
        const geometry_msgs::PoseStamped& target_pose,
        const std::string& controller = "FTCPlanner"
    );

    /**
     * Called ONCE on state enter.
     */
    virtual void enter() = 0;

public:

    virtual std::string state_name() = 0;
    virtual std::string sub_state_name() {
        return "";
    }

    bool hasGoodGPS()
    {
        return isGPSGood;
    }

    bool hasFixedGPS()
    {
        return isGPSFixed;
    }

    void setGoodGPS(bool isGood) {
        if (isGood && !isGPSGood) {
            ROS_INFO_STREAM("bahavior: GPS is now good");
        } else if (!isGood && isGPSGood) {
            ROS_WARN_STREAM("behavior: GPS is now bad");
        }
        isGPSGood = isGood;
    }

    void setFixedGPS(bool isFixed) {
        if (isFixed && !isGPSFixed) {
            ROS_INFO_STREAM("bahavior: GPS is now fixed");
        } else if (!isFixed && isGPSFixed) {
            ROS_WARN_STREAM("behavior: GPS is now not fixed");
        }
        isGPSFixed = isFixed;
    }
    
    void requestContinue()
    {
        requested_continue_flag = true;
    }

    void requestPause()
    {
        requested_pause_flag = true;
    }

    void requestCrashRecovery()
    {
        requested_crash_recovery_flag = true;
    }

    void setPause()
    {
        paused = true;
    }

    void setContinue()
    {
        paused = false;
        requested_continue_flag = false;
        requested_pause_flag = false;
    }

    void start(mower_logic::MowerLogicConfig &c, std::shared_ptr<sSharedState> s) {
        ROS_INFO_STREAM("");
        ROS_INFO_STREAM("");
        ROS_INFO_STREAM("--------------------------------------");
        ROS_INFO_STREAM("- Entered state: " << state_name());
        ROS_INFO_STREAM("--------------------------------------");
        aborted = false;
        paused = false;
        requested_continue_flag = false;
        requested_pause_flag = false;
        requested_crash_recovery_flag = false;
        this->config = c;
        this->shared_state = std::move(s);
        startTime = ros::Time::now();
        isGPSGood = false;
        sub_state = 0;
        enter();
    }

    /**
     * Execute the behavior. This call should block until the behavior is executed fully.
     * @returns the pointer to the next behavior (can return itself).
     */
    virtual Behavior *execute() = 0;

    /**
     * Called ONCE before state exits
     */
    virtual void exit() = 0;

    /**
     * Reset the internal state of the behavior.
     */
    virtual void reset() = 0;

    /**
     * If called, save state internally and return the execute() method asap.
     * Execution should resume on the next execute() call.
     */
    void abort() {
        if(!aborted) {
            ROS_INFO_STREAM( "- Behaviour.h: abort() called");
        }
        aborted = true;
    }

    // Return true, if this state needs absolute positioning.
    // The state will be aborted if GPS is lost and resumed at some later point in time.
    virtual bool needs_gps() = 0;

    // return true, if the mower motor should currently be running.
    virtual bool mower_enabled() = 0;

    // return true to redirect joystick speeds to the controller
    virtual bool redirect_joystick() = 0;


    virtual void command_home() = 0;
    virtual void command_start() = 0;
    virtual void command_s1() = 0;
    virtual void command_s2() = 0;
    virtual void command_drive() = 0;

    virtual uint8_t get_sub_state() = 0;
    virtual uint8_t get_state() = 0;

    virtual void handle_action(std::string action) = 0;
};

#endif //SRC_BEHAVIOR_H
