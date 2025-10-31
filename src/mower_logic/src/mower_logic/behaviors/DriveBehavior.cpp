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
#include "DriveBehavior.h"
#include "DockingBehavior.h"

extern bool setGPS(bool enabled);

DriveBehavior DriveBehavior::INSTANCE;

void DriveBehavior::set_point(const geometry_msgs::PoseStamped& position) {
    target_pose = position;
    has_target = true;
    should_dock = false;
}

std::string DriveBehavior::state_name() {
    return "DRIVE";
}

Behavior *DriveBehavior::execute() {
    if (!has_target) {
        ROS_ERROR_STREAM("DriveBehavior: No target set, returning to idle.");
        return &IdleBehavior::INSTANCE;
    }

    setGPS(true);
    while(!isGPSGood) {
        if (aborted) {
            ROS_WARN_STREAM("DriveBehavior: Aborted while waiting for GPS.");
            if (should_dock) {
                return &DockingBehavior::INSTANCE;
            }
            return &IdleBehavior::INSTANCE;
        }
        ROS_WARN_STREAM("DriveBehavior: Waiting for good GPS");
        ros::Duration(1.0).sleep();
    }

    ROS_INFO_STREAM("DriveBehavior: Driving to target position (" 
                    << target_pose.pose.position.x << ", " 
                    << target_pose.pose.position.y << ")");

    bool success = drive_to_position(target_pose, "FTCPlanner");

    if (success) {
        ROS_INFO_STREAM("DriveBehavior: Successfully reached target position.");
    } else {
        ROS_ERROR_STREAM("DriveBehavior: Failed to reach target position.");
    }

    if (should_dock) {
        return &DockingBehavior::INSTANCE;
    }

    return &IdleBehavior::INSTANCE;
}

void DriveBehavior::enter() {
    paused = aborted = false;
    should_dock = false;
}

void DriveBehavior::exit() {
    // Nothing to clean up
}

void DriveBehavior::reset() {
    has_target = false;
    should_dock = false;
}

bool DriveBehavior::needs_gps() {
    return true;
}

bool DriveBehavior::mower_enabled() {
    // No mower during driving
    return false;
}

void DriveBehavior::command_home() {
    // Set flag to dock after abort
    should_dock = true;
    this->abort();
}

void DriveBehavior::command_start() {
    // Not applicable
}

void DriveBehavior::command_s1() {
    // Not applicable
}

void DriveBehavior::command_s2() {
    // Not applicable
}

void DriveBehavior::command_drive() {
    // Not applicable
}

bool DriveBehavior::redirect_joystick() {
    return false;
}

uint8_t DriveBehavior::get_sub_state() {
    return 0;
}

uint8_t DriveBehavior::get_state() {
    return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

void DriveBehavior::handle_action(std::string action) {
    // No actions for now
}
