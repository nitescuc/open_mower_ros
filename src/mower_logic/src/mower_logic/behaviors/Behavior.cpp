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
#include "Behavior.h"
#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/MoveBaseAction.h>
#include "mower_msgs/Status.h"

extern mower_msgs::Status getStatus();
extern void stopMoving();
extern bool isEmergencyMode();
extern int getCurrentPathProgress();
extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *mbfClient;

bool Behavior::drive_to_position(const geometry_msgs::PoseStamped& target_pose, const std::string& controller, const std::function<int(int)> &state_cb) {
    mbf_msgs::MoveBaseGoal moveBaseGoal;
    moveBaseGoal.target_pose = target_pose;
    moveBaseGoal.controller = controller;
    return execute_goal(mbfClient, moveBaseGoal, state_cb);
}

bool Behavior::execute_goal(actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *client, mbf_msgs::MoveBaseGoal goal, const std::function<int(int)> &state_cb) {
    client->sendGoal(goal);

    bool goalSuccess = false;
    bool waitingForResult = true;

    ros::Rate r(10);

    // we can assume the last_state is current since we have a security timer
    int old_index = -1;
    ros::Time last_index_time = ros::Time::now();
    while (waitingForResult) {

        r.sleep();

        const auto last_status = getStatus();
        auto mbfState = client->getState();

        // Allow external callback to influence execution based on state
        if (state_cb) {
            int cb = 0;
            try {
                cb = state_cb(mbfState.state_);
            } catch (const std::exception &e) {
                ROS_ERROR_STREAM_THROTTLE(5, "Behavior: state callback threw exception: " << e.what());
                cb = -1; // treat as error
            }
            if (cb < 0) {
                ROS_WARN_STREAM("Behavior: state callback requested ABORT (cb<0), cancelling goal.");
                client->cancelAllGoals();
                stopMoving();
                goalSuccess = false;
                waitingForResult = false;
                continue;
            } else if (cb > 0) {
                ROS_INFO_STREAM("Behavior: state callback signaled SUCCESS (cb>0), cancelling goal.");
                client->cancelAllGoals();
                stopMoving();
                goalSuccess = true;
                waitingForResult = false;
                continue;
            }
            // cb == 0 -> no change, continue with normal logic
        }

        if(aborted) {
            ROS_INFO_STREAM("Goal execution aborted.");
            client->cancelAllGoals();
            stopMoving();
            goalSuccess = false;
            waitingForResult = false;
            continue;
        }

        int index = getCurrentPathProgress();
        switch (mbfState.state_) {
            case actionlib::SimpleClientGoalState::ACTIVE:
            case actionlib::SimpleClientGoalState::PENDING:
                // currently moving. Cancel as soon as we're in the station
                if (last_status.v_charge > 5.0) {
                    ROS_INFO_STREAM("Got a voltage of " << last_status.v_charge << " V. Cancelling goal.");
                    client->cancelAllGoals();
                    stopMoving();
                    goalSuccess = true;
                    waitingForResult = false;
                    continue;
                }
                if (index != old_index) {
                    last_index_time = ros::Time::now();
                    old_index = index;
                } else {
                    if (!this->hasGoodGPS() || isEmergencyMode()) {
                        if (!this->hasGoodGPS())
                            ROS_WARN_STREAM_THROTTLE(10, "Behavior: (execute_goal) - No GPS signal, waiting.");
                        if (isEmergencyMode())
                            ROS_WARN_STREAM_THROTTLE(10, "Behavior: (execute_goal) - Emergency mode, waiting.");
                        last_index_time = ros::Time::now();
                    } else {
                        if ((ros::Time::now() - last_index_time).toSec() > 30.0) {
                            ROS_ERROR_STREAM("Behavior: (execute_goal) - No progress for 30 seconds, stopping path execution. HasGoodGPS=" << this->hasGoodGPS());
                            mbfClient->cancelAllGoals();
                            stopMoving();
                            goalSuccess = false;
                            waitingForResult = false;
                            continue;
                        }
                    }
                }
                ROS_INFO_STREAM_THROTTLE(5, "Behavior: Goal Progress: " << index);
                
                break;
            case actionlib::SimpleClientGoalState::SUCCEEDED:
                // we stopped moving because the path has ended. check, if we have docked successfully
                if (last_status.v_charge > 5.0) {
                    ROS_INFO_STREAM("Goal stopped, because we reached end pose. Voltage was " << last_status.v_charge << " V.");
                    client->cancelAllGoals();
                    stopMoving();
                } else {
                    ROS_INFO_STREAM("Behavior: Goal reached");
                }
                goalSuccess = true;
                waitingForResult = false;
                break;
            default:
                ROS_WARN_STREAM("Some error during path execution. Goal failed. status value was: "
                                        << mbfState.state_);
                waitingForResult = false;
                stopMoving();
                break;
        }
    }
    return goalSuccess;
}
