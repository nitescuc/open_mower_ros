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
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <costmap_2d/cost_values.h>
#include "mower_msgs/Status.h"

extern mower_msgs::Status getStatus();
extern void stopMoving();
extern bool isEmergencyMode();
extern int getCurrentPathProgress();
extern bool setGPSRtkFloat(bool enabled);
extern nav_msgs::OccupancyGrid::ConstPtr getCostmap();
extern nav_msgs::Odometry getOdometry();
extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *mbfClient;

int Behavior::on_progress(int state) {
    const auto last_status = getStatus();
    
    switch (state) {
        case actionlib::SimpleClientGoalState::ACTIVE:
        case actionlib::SimpleClientGoalState::PENDING:
            // currently moving. Cancel as soon as we're in the station
            if (last_status.v_charge > 5.0) {
                ROS_INFO_STREAM("Got a voltage of " << last_status.v_charge << " V. Cancelling goal.");
                return 1; // Signal success (charging detected)
            }
            // Check abort flag first
            if(aborted) {
                ROS_INFO_STREAM("Goal execution aborted.");
                return PROGRESS_ERROR_UNRECOVERABLE; // Unrecoverable abort
            }
            // pause will do like abort, then the behavior can handle the pause request properly
            if (requested_pause_flag) {
                ROS_INFO_STREAM("Behavior: (on_progress) - PAUSE was requested - stopping path execution.");
                return PROGRESS_ERROR_UNRECOVERABLE; // treat as unrecoverable to avoid retries
            }
            // crash recovery requested?
            if (requested_crash_recovery_flag) {
                ROS_WARN_STREAM("Behavior: (on_progress) - CRASH RECOVERY was requested - stopping path execution and waiting 2sec to calm down.");
                // debounce
                ros::Duration(2.0).sleep();
                requested_crash_recovery_flag = false;
                return PROGRESS_ERROR_RECOVERABLE; // treat as recoverable to allow retry
            }
            // Check for progress timeout
            {
                int index = getCurrentPathProgress();
                if (index != progress_old_index) {
                    progress_last_index_time = ros::Time::now();
                    progress_old_index = index;
                } else {
                    if (!this->hasGoodGPS() || isEmergencyMode()) {
                        if (!this->hasGoodGPS())
                            ROS_WARN_STREAM_THROTTLE(10, "Behavior: (on_progress) - No GPS signal, waiting.");
                        if (isEmergencyMode())
                            ROS_WARN_STREAM_THROTTLE(10, "Behavior: (on_progress) - Emergency mode, waiting.");
                        progress_last_index_time = ros::Time::now();
                    } else {
                        if ((ros::Time::now() - progress_last_index_time).toSec() > 30.0) {
                            ROS_ERROR_STREAM("Behavior: (on_progress) - No progress for 30 seconds, stopping path execution. HasGoodGPS=" << this->hasGoodGPS());
                            return PROGRESS_ERROR_RECOVERABLE; // treat as recoverable to allow retry
                        }
                    }
                }
                ROS_INFO_STREAM_THROTTLE(5, "Behavior: Goal Progress: " << index);
            }
            break;
            
        case actionlib::SimpleClientGoalState::SUCCEEDED:
            // we stopped moving because the path has ended. check, if we have docked successfully
            if (last_status.v_charge > 5.0) {
                ROS_INFO_STREAM("Goal stopped, because we reached end pose. Voltage was " << last_status.v_charge << " V.");
            } else {
                ROS_INFO_STREAM("Behavior: Goal reached");
            }
            return 1; // Signal success
            
        default:
            ROS_WARN_STREAM("Some error during path execution. Goal failed. status value was: " << state);
            return PROGRESS_ERROR_RECOVERABLE; // generic recoverable error
    }
    
    return 0; // Continue normally
}

bool Behavior::drive_to_position(const geometry_msgs::PoseStamped& target_pose, const std::string& controller, int retry_count) {
    mbf_msgs::MoveBaseGoal moveBaseGoal;
    moveBaseGoal.target_pose = target_pose;
    moveBaseGoal.controller = controller;
    return execute_goal(mbfClient, moveBaseGoal, retry_count);
}

bool Behavior::waitForFixedGPS(double wait_time_seconds) {
    // Require clean RTK fix (disable float)
    setGPSRtkFloat(false);
    
    auto start = ros::Time::now();
    while (start + ros::Duration(wait_time_seconds, 0) > ros::Time::now()) {
        if (!ros::ok() || aborted) {
            return false;
        }
        if (!isGPSFixed) {
            start = ros::Time::now();
            ROS_WARN_STREAM("Waiting for fixed GPS");
        } else {
            ROS_INFO_STREAM("GPS is fixed");
        }
        ros::Duration(1.0).sleep();
    }
    return true;
}

bool Behavior::isCurrentPositionLethal() {
    auto costmap = getCostmap();
    if (!costmap) {
        ROS_WARN_STREAM_THROTTLE(5, "Behavior::isCurrentPositionLethal - No costmap available");
        return false;
    }
    
    auto odom = getOdometry();
    double robot_x = odom.pose.pose.position.x;
    double robot_y = odom.pose.pose.position.y;
    
    // Convert world coordinates to map coordinates
    unsigned int mx, my;
    double resolution = costmap->info.resolution;
    double origin_x = costmap->info.origin.position.x;
    double origin_y = costmap->info.origin.position.y;
    
    mx = (unsigned int)((robot_x - origin_x) / resolution);
    my = (unsigned int)((robot_y - origin_y) / resolution);
    
    // Check if coordinates are within map bounds
    if (mx >= costmap->info.width || my >= costmap->info.height) {
        ROS_WARN_STREAM_THROTTLE(5, "Behavior::isCurrentPositionLethal - Position (" << robot_x << ", " << robot_y 
                                  << ") is outside costmap bounds");
        return false;
    }
    
    unsigned int index = my * costmap->info.width + mx;
    if (index >= costmap->data.size()) {
        ROS_WARN_STREAM_THROTTLE(5, "Behavior::isCurrentPositionLethal - Index out of bounds");
        return false;
    }
    
    unsigned char cost = costmap->data[index];
    bool is_lethal = (cost >= costmap_2d::LETHAL_OBSTACLE);
    
    if (is_lethal) {
        ROS_WARN_STREAM("Behavior::isCurrentPositionLethal - Robot at position (" << robot_x << ", " << robot_y 
                       << ") has lethal cost: " << (int)cost);
    } else {
        ROS_INFO_STREAM_THROTTLE(10, "Behavior::isCurrentPositionLethal - Robot at position (" << robot_x << ", " << robot_y 
                       << ") has non-lethal cost: " << (int)cost);
    }
    
    return is_lethal;
}

bool Behavior::execute_goal(actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *client, mbf_msgs::MoveBaseGoal goal, int retry_count) {
    // Ensure at least one attempt
    if (retry_count < 1) retry_count = 1;

    for (int attempt = 1; attempt <= retry_count; ++attempt) {
        if (aborted) {
            ROS_WARN_STREAM("Behavior::execute_goal - Aborted before sending goal, not retrying.");
            return false;
        }

        ROS_INFO_STREAM("Behavior::execute_goal - Sending goal, attempt " << attempt << "/" << retry_count);
        client->sendGoal(goal);

        bool goalSuccess = false;
        bool waitingForResult = true;
        bool unrecoverable_error = false;

        ros::Rate r(10);

        // Initialize progress tracking variables
        progress_old_index = -1;
        progress_last_index_time = ros::Time::now();

        while (waitingForResult) {
            r.sleep();

            auto mbfState = client->getState();

            // Allow derived classes to influence execution based on state
            int progress_result = 0;
            try {
                progress_result = this->on_progress(mbfState.state_);
            } catch (const std::exception &e) {
                ROS_ERROR_STREAM_THROTTLE(5, "Behavior: on_progress threw exception: " << e.what());
                progress_result = -1; // treat as error
            }

            if (progress_result < 0) {
                ROS_WARN_STREAM("Behavior: on_progress requested ABORT (<0), cancelling goal.");
                client->cancelAllGoals();
                stopMoving();
                goalSuccess = false;
                waitingForResult = false;
                if (progress_result == PROGRESS_ERROR_UNRECOVERABLE) {
                    unrecoverable_error = true;
                }
            } else if (progress_result > 0) {
                ROS_INFO_STREAM("Behavior: on_progress signaled SUCCESS (>0), cancelling goal.");
                client->cancelAllGoals();
                stopMoving();
                goalSuccess = true;
                waitingForResult = false;
            }
            // progress_result == 0 -> continue normally
        }

        if (goalSuccess) {
            return true;
        }

        if (unrecoverable_error) {
            ROS_WARN_STREAM("Behavior::execute_goal - Unrecoverable error after attempt " << attempt << ", not retrying.");
            return false;
        }

        if (attempt < retry_count) {
            ROS_WARN_STREAM("Behavior::execute_goal - Goal failed, retrying (" << (attempt + 1) << "/" << retry_count << ") after short delay.");
            ros::Duration(1.0).sleep();
        }
    }

    // All attempts failed
    return false;
}
