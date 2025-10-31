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
#ifndef SRC_DRIVEBEHAVIOR_H
#define SRC_DRIVEBEHAVIOR_H

#include "Behavior.h"
#include "IdleBehavior.h"
#include <geometry_msgs/PoseStamped.h>

class DriveBehavior : public Behavior {
public:
    static DriveBehavior INSTANCE;

private:
    geometry_msgs::PoseStamped target_pose;
    bool has_target;
    bool should_dock;

public:
    /**
     * Set the target position to drive to.
     * @param position The target pose to drive to
     */
    void set_point(const geometry_msgs::PoseStamped& position);

    std::string state_name() override;

    Behavior *execute() override;

    void enter() override;

    void exit() override;

    void reset() override;

    bool needs_gps() override;

    bool mower_enabled() override;

    void command_home() override;

    void command_start() override;

    void command_s1() override;

    void command_s2() override;

    void command_drive() override;

    bool redirect_joystick() override;

    uint8_t get_sub_state() override;

    uint8_t get_state() override;

    void handle_action(std::string action) override;
};

#endif //SRC_DRIVEBEHAVIOR_H
