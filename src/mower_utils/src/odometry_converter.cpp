#include "ros/ros.h"
#include "xbot_msgs/WheelTick.h"
#include <nav_msgs/Odometry.h>
#include "xbot_msgs/AbsolutePose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "mower_utils/GPSControlSrv.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_listener.h>

ros::Publisher imu_pub, filtered_imu_pub, odometry_pub, pose_pub;
tf2_ros::Buffer tfBuffer;

sensor_msgs::Imu imu;
bool has_ticks;
xbot_msgs::WheelTick last_ticks;
nav_msgs::Odometry odometry;
geometry_msgs::PoseWithCovarianceStamped pose;
std::string odometry_frame_id = "odom";
double vx = 0.0;

double cov_factor_pos = 10;
double cov_factor_ori = 100;
double cov_factor_float_pos = 500;
double cov_factor_float_ori = 5000;
double orientation_min_speed = 0.01;
double antenna_offset_x = 0.15;
double antenna_offset_y = 0.0;
double min_position_accuracy = 0.05;
double float_damping_factor = 10.0;

bool has_gyro;
sensor_msgs::Imu filtered_imu;
ros::Time gyro_calibration_start;
double gyro_offset;
int gyro_offset_samples;
double accelerometer_offset;

ros::Time last_gps_fixed_time(0.0), last_gps_float_time(0.0);

bool gps_enabled = true;

bool setGpsState(mower_utils::GPSControlSrvRequest &req, mower_utils::GPSControlSrvResponse &res) {
    gps_enabled = req.gps_enabled;
    return true;
}

void onGPS(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
    if (!gps_enabled) {
        return;
    }

    double damping = 0.0;
    bool is_fixed = false;
    if ((msg->flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT) == xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT) {
        // if it's float since some time now, increase damping factor
        if (last_gps_fixed_time < last_gps_float_time) {
            damping = float_damping_factor * (ros::Time::now() - last_gps_float_time).toSec();
        }
        last_gps_float_time = ros::Time::now();
    } else {
        // if the fixed signal is recovered since more than 10 seconds, and the fixed signal time is recent, then we consider it fixed
        if (last_gps_float_time.isZero() || last_gps_float_time < last_gps_fixed_time) {
            if ((ros::Time::now() - last_gps_float_time).toSec() > 10.0 && (ros::Time::now() - last_gps_fixed_time).toSec() < 1.0) {
                is_fixed = true;
            }
        }
        last_gps_fixed_time = ros::Time::now();
    }
    // min_position_accuracy is the minimum accuracy of the GPS position in meters, if <= 0.0 it requires RTK_FIXED
    if (min_position_accuracy <= 0.0) {
        if (!is_fixed) {
            ROS_WARN_STREAM_THROTTLE(60, "odom_converter: GPS not RTK fixed");
            return;
        }
    } else {
        if (msg->position_accuracy > min_position_accuracy || (((ros::Time::now() - last_gps_fixed_time).toSec() > 60.0) && !is_fixed)) {
            ROS_WARN_STREAM_THROTTLE(60, "odom_converter: GPS position accuracy is too low: " << msg->position_accuracy << "; senconds since last fixed:" << (ros::Time::now() - last_gps_fixed_time).toSec());
            return;
        }
    }

    tf2::Quaternion q;
    double heading = msg->motion_heading;
    // if reversing, flip the heading
    if (vx < 0.0) {
        heading += M_PI;
        if (heading > 2*M_PI) {
            heading -= 2*M_PI;
        }
    }
    q.setRPY(0, 0, heading);

    // convert pose from "gps" frame to "map" frame
    geometry_msgs::PoseStamped pose_map;
    try {
        // get current yaw from map to base_link
        geometry_msgs::TransformStamped transform_stamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);

        // Convert quaternion to RPY
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        pose_map.pose = msg->pose.pose;
        pose_map.pose.position.x -= antenna_offset_x * cos(yaw);
        pose_map.pose.position.y -= antenna_offset_x * sin(yaw);
        pose_map.pose.position.z = 0;
        // ROS_INFO_STREAM("transform gps to map; gps pose: " << msg->pose.pose << "; map pose: " << pose_map.pose);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("om_mower_logic map/base_link: %s", ex.what());
        return;
    }


    pose.header.stamp = msg->header.stamp;
    pose.header.seq++;
    pose.header.frame_id = "map";
    //pose.pose = msg->pose;
    pose.pose.pose = pose_map.pose;
    pose.pose.covariance = msg->pose.covariance;
    pose.pose.covariance[21] = 0.0;
    pose.pose.covariance[28] = 0.0;
    pose.pose.pose.position.z = 0.0;
    pose.pose.pose.orientation = tf2::toMsg(q);
    if (is_fixed) {
        pose.pose.covariance[0] = pose.pose.covariance[0] * cov_factor_pos;
        pose.pose.covariance[7] = pose.pose.covariance[7] * cov_factor_pos;
        pose.pose.covariance[14] = pose.pose.covariance[14] * cov_factor_pos;
        pose.pose.covariance[35] = msg->orientation_accuracy * msg->orientation_accuracy * cov_factor_ori;
    } else {
        pose.pose.covariance[0] = pose.pose.covariance[0] * (cov_factor_float_pos + damping);
        pose.pose.covariance[7] = pose.pose.covariance[7] * (cov_factor_float_pos + damping);
        pose.pose.covariance[14] = pose.pose.covariance[14] * (cov_factor_float_pos + damping);
        pose.pose.covariance[35] = msg->orientation_accuracy * msg->orientation_accuracy * cov_factor_float_ori;
    }

    pose_pub.publish(pose);

    // publish odometry only if the speed is relevant
    if(std::sqrt(std::pow(msg->motion_vector.x, 2)+std::pow(msg->motion_vector.y, 2)) >= orientation_min_speed) {
        imu.header.stamp = msg->header.stamp;
        imu.header.seq++;
        imu.header.frame_id = "gps";
        imu.orientation = pose.pose.pose.orientation;
        imu.orientation_covariance[8] = pose.pose.covariance[35];

        imu_pub.publish(imu);
    }
}

void onWheelTicks(const xbot_msgs::WheelTick::ConstPtr &msg) {
    if(!has_ticks) {
        last_ticks = *msg;
        has_ticks = true;
        return;
    }
    double dt = (msg->stamp - last_ticks.stamp).toSec();

    // double d_wheel_l = (double) (msg->wheel_ticks_rl - last_ticks.wheel_ticks_rl) * (1/(double)msg->wheel_tick_factor);
    // double d_wheel_r = (double) (msg->wheel_ticks_rr - last_ticks.wheel_ticks_rr) * (1/(double)msg->wheel_tick_factor);
    double d_wheel_l = (double) (msg->wheel_ticks_rl - last_ticks.wheel_ticks_rl) * (1/330.0);
    double d_wheel_r = (double) (msg->wheel_ticks_rr - last_ticks.wheel_ticks_rr) * (1/330.0);

    if(msg->wheel_direction_rl) {
        d_wheel_l *= -1.0;
    }
    if(msg->wheel_direction_rr) {
        d_wheel_r *= -1.0;
    }

    double d_ticks = (d_wheel_l + d_wheel_r) / 2.0;
    vx = d_ticks / dt;

    last_ticks = *msg;

    // detect wheel sliping

    // limit wheel angular speed
    double angular_speed = (d_wheel_r - d_wheel_l)/(dt * 0.33);
    if(abs(angular_speed) > 4) {
        ROS_WARN_STREAM("got inconsistent angular_speed (" << angular_speed << ") - droping vx");
        return;
    } 

    // consider max possible robot speed of 0.50 (TODO get it from parameters)
    if(abs(vx) > 1.0) {
        ROS_WARN_STREAM("got vx > 1.0 (" << vx << ") - dropping measurement");
        return;
    }

    // publish odometry
    odometry.header.stamp = msg->stamp;
    odometry.header.seq++;
    odometry.header.frame_id = odometry_frame_id;
    odometry.child_frame_id = "base_link";
    
    odometry.twist.twist.linear.x = vx;
    odometry.twist.twist.angular.z = angular_speed;
    odometry.twist.covariance[0] = 0.0001;
    odometry.twist.covariance[35] = 0.05;

    odometry_pub.publish(odometry);
}

void onImu(const sensor_msgs::Imu::ConstPtr &msg) {
    if(!has_gyro) {
        if (gyro_offset_samples == 0) {
            ROS_INFO_STREAM("Started gyro calibration");
            gyro_calibration_start = msg->header.stamp;
            gyro_offset = 0;
            accelerometer_offset = 0;
        }
        gyro_offset += msg->angular_velocity.z;
        gyro_offset_samples++;
        accelerometer_offset += msg->linear_acceleration.x;
        if ((msg->header.stamp - gyro_calibration_start).toSec() < 5) {
            return;
        }
        has_gyro = true;
        if (gyro_offset_samples > 0) {
            gyro_offset /= gyro_offset_samples;
            accelerometer_offset /= gyro_offset_samples;
        } else {
            gyro_offset = 0;
            accelerometer_offset = 0;
        }
        gyro_offset_samples = 0;
        ROS_INFO_STREAM("Odom Convertor: Calibrated gyro offset: " << gyro_offset);
        ROS_INFO_STREAM("Odom Convertor: Calibrated accelerometer offset: " << accelerometer_offset);
    }

    filtered_imu = *msg;
    filtered_imu.angular_velocity.z -= gyro_offset;
    filtered_imu.linear_acceleration.x -= accelerometer_offset;

    filtered_imu_pub.publish(filtered_imu);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_converter");

    has_gyro = false;

    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    ros::Subscriber wheel_tick_sub = paramNh.subscribe("/mower/wheel_ticks", 10, onWheelTicks);
    ros::Subscriber gps_sub = paramNh.subscribe("/xbot_driver_gps/xb_pose", 10, onGPS);
    ros::Subscriber raw_imu_sub = paramNh.subscribe("/imu/data_raw", 10, onImu);
    odometry_pub = paramNh.advertise<nav_msgs::Odometry>("odom", 10);
    imu_pub = paramNh.advertise<sensor_msgs::Imu>("orientation", 10);
    filtered_imu_pub = paramNh.advertise<sensor_msgs::Imu>("filtered_imu", 10);
    pose_pub = paramNh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 10);

    paramNh.param("cov_factor_pos", cov_factor_pos, 10.0);
    paramNh.param("cov_factor_ori", cov_factor_ori, 100.0);
    paramNh.param("cov_factor_float_pos", cov_factor_float_pos, 500.0);
    paramNh.param("cov_factor_float_ori", cov_factor_float_ori, 5000.0);
    paramNh.param("orientation_min_speed", orientation_min_speed, 0.01);
    paramNh.param("antenna_offset_x", antenna_offset_x, 0.0);
    paramNh.param("antenna_offset_y", antenna_offset_y, 0.0);
    paramNh.param("min_position_accuracy", min_position_accuracy, 0.0);
    paramNh.param("float_damping_factor", float_damping_factor, 10.0);


    ros::ServiceServer gps_service = n.advertiseService("odom_converter/set_gps_state", setGpsState);

    ros::spin();

    return 0;
}