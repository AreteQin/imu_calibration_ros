/**
 * @file   ImuSimulator.cpp
 * @brief  Tool to simulate imu data, ref:
 * https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model.
 * @author Rick Liu
 */

// std, eigen and boost
#include <boost/filesystem.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <ctime>
#include <fstream>
#include <set>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

// ROS
#include <ros/node_handle.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>

//#include "allan_variance_ros/yaml_parsers.hpp"

using Vec3d = Eigen::Vector3d;

const float eps = 1e-4;

Vec3d RandomNormalDistributionVector(double sigma) {
    static boost::mt19937 rng;
    static boost::normal_distribution<> nd(0, 1);
    return {sigma * nd(rng), sigma * nd(rng), sigma * nd(rng)};
}

template<typename S, typename T>
void FillROSVector3d(const S &from, T &to) {
    to.x = from.x();
    to.y = from.y();
    to.z = from.z();
}

// calculate the rotation matrix R_{j-1,j} from one gyro measurement and corresponding right Jacobian
// equation (5.1) and (5.6.1)
Eigen::Matrix3d IntegratedRotation(const Eigen::Vector3d &angVel, const double &time) {
    const double x = angVel(0) * time;
    const double y = angVel(1) * time;
    const double z = angVel(2) * time;
    Eigen::Vector3d v;
    v << x, y, z;
    Eigen::Matrix3d W = Sophus::SO3d::hat(v);

    const float d2 = x * x + y * y + z * z;
    const float d = sqrt(d2);
    Eigen::Matrix3d deltaR;
    if (d < eps) {
        // equation (1.3)
        deltaR = Eigen::Matrix3d::Identity() + W;
    } else {
        // using Rodrigues' formula
        deltaR = Eigen::Matrix3d::Identity() + W * sin(d) / d + W * W * (1.0f - cos(d)) / d2;
    }
    return deltaR;
}

// 将不是标准正交的旋转矩阵单位正交化
Eigen::Matrix3d NormalizeRotation(const Eigen::Matrix3d &R) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
}

class ImuSimulator {
public:
    ImuSimulator(std::string output_path)
            : bag_output_(output_path, rosbag::bagmode::Write) {
//        auto yaml_config = loadYamlFile(config_file);

//        get(yaml_config, "accelerometer_noise_density",
//            accelerometer_noise_density_);
//        get(yaml_config, "accelerometer_random_walk", accelerometer_random_walk_);
//        get(yaml_config, "accelerometer_bias_init", accelerometer_bias_init_);
//
//        get(yaml_config, "gyroscope_noise_density", gyroscope_noise_density_);
//        get(yaml_config, "gyroscope_random_walk", gyroscope_random_walk_);
//        get(yaml_config, "gyroscope_bias_init", gyroscope_bias_init_);

//        get(yaml_config, "rostopic", rostopic_);
        ROS_INFO_STREAM("rostopic: " << rostopic_);
//        get(yaml_config, "update_rate", update_rate_);
        ROS_INFO_STREAM("update_rate: " << update_rate_);
//        get(yaml_config, "sequence_time", init_static_period_);
        ROS_INFO_STREAM("sequence_time: " << init_static_period_);
    }

    virtual ~ImuSimulator() { bag_output_.close(); }

    void GenerateDataWithIntervals() {
        ROS_INFO_STREAM("Generating IMU data ...");

        double dt = 1 / update_rate_;

        // clang-format off
        ros::Time start_time(1.0);
        Vec3d accelerometer_bias = Vec3d::Constant(accelerometer_bias_init_);
        Vec3d gyroscope_bias = Vec3d::Constant(gyroscope_bias_init_);
        Vec3d accelerometer_real = {0, 0, 9.81};
        Vec3d gyroscope_real = Vec3d::Zero();

        for (int64_t i = 0; i < init_static_period_ * update_rate_; ++i) {

            // Break if requested by user
            if (!ros::ok()) {
                break;
            }

            // Reference: https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
            accelerometer_bias += RandomNormalDistributionVector(accelerometer_random_walk_) * sqrt(dt);
            gyroscope_bias += RandomNormalDistributionVector(gyroscope_random_walk_) * sqrt(dt);

            Vec3d acc_measure = accelerometer_real + accelerometer_bias +
                                RandomNormalDistributionVector(accelerometer_noise_density_) / sqrt(dt);
            Vec3d gyro_measure = gyroscope_real + gyroscope_bias +
                                 RandomNormalDistributionVector(gyroscope_noise_density_) / sqrt(dt);

            sensor_msgs::Imu msg;
            msg.header.stamp = start_time + ros::Duration(1, 0) * (i / update_rate_);
            msg.header.seq = i;
            FillROSVector3d(acc_measure, msg.linear_acceleration);
            FillROSVector3d(gyro_measure, msg.angular_velocity);

//            FillROSVector3d(accelerometer_real, msg.linear_acceleration);
//            FillROSVector3d(gyroscope_real, msg.angular_velocity);

            bag_output_.write(rostopic_, msg.header.stamp, msg);
        }

        uint64_t total_imu_data = (init_static_period_ + (1 + static_interval_time_) * num_interval_) * update_rate_;
//        ROS_INFO_STREAM("total_imu_data: " << total_imu_data);
        for (uint64_t i = init_static_period_ * update_rate_; i < total_imu_data; ++i) {
            if (i % static_cast<uint>(update_rate_ * (1 + static_interval_time_)) < update_rate_) {
//                ROS_INFO_STREAM("rotating");
                gyroscope_real = {2, 2, 2};
//                ROS_INFO_STREAM("rotation matrix: "<<NormalizeRotation(IntegratedRotation(gyroscope_real, dt)));
                accelerometer_real = NormalizeRotation(IntegratedRotation(gyroscope_real, dt)) * accelerometer_real;
            } else {
//                ROS_INFO_STREAM("static");
                gyroscope_real = {0, 0, 0};
//                ROS_INFO_STREAM("rotation matrix: "<<IntegratedRotation(gyroscope_real, dt));
            }
            accelerometer_bias += RandomNormalDistributionVector(accelerometer_random_walk_) * sqrt(dt);
            gyroscope_bias += RandomNormalDistributionVector(gyroscope_random_walk_) * sqrt(dt);

            Vec3d acc_measure = accelerometer_real + accelerometer_bias +
                                RandomNormalDistributionVector(accelerometer_noise_density_) / sqrt(dt);
            Vec3d gyro_measure = gyroscope_real + gyroscope_bias +
                                 RandomNormalDistributionVector(gyroscope_noise_density_) / sqrt(dt);

            sensor_msgs::Imu msg;
            msg.header.seq = i;
//            ROS_INFO_STREAM("msg.header.seq: " << msg.header.seq);
            msg.header.stamp = start_time + ros::Duration(1, 0) * (i / update_rate_);
            FillROSVector3d(acc_measure, msg.linear_acceleration);
            FillROSVector3d(gyro_measure, msg.angular_velocity);
//            FillROSVector3d(accelerometer_real, msg.linear_acceleration);
//            FillROSVector3d(gyroscope_real, msg.angular_velocity);

            bag_output_.write(rostopic_, msg.header.stamp, msg);
        }

//        for (uint interval_index = 0; interval_index < num_interval_; ++interval_index) {

//            gyroscope_real = Vec3d::Ones();

//            for (int64_t i = 0; i < update_rate_; ++i) {
//
//                // Break if requested by user
//                if (!ros::ok()) {
//                    break;
//                }
//
//                accelerometer_real = IntegratedRotation(gyroscope_real, dt) * accelerometer_real;
//
//                accelerometer_bias += RandomNormalDistributionVector(accelerometer_random_walk_) * sqrt(dt);
//                gyroscope_bias += RandomNormalDistributionVector(gyroscope_random_walk_) * sqrt(dt);
//
//                Vec3d acc_measure = accelerometer_real + accelerometer_bias +
//                                    RandomNormalDistributionVector(accelerometer_noise_density_) / sqrt(dt);
//                Vec3d gyro_measure = gyroscope_real + gyroscope_bias +
//                                     RandomNormalDistributionVector(gyroscope_noise_density_) / sqrt(dt);
//
//                sensor_msgs::Imu msg;
//                msg.header.seq = init_static_period_ * update_rate_ + interval_index * update_rate_ + i;
//                ROS_INFO_STREAM("msg.header.seq: " << msg.header.seq);
//                msg.header.stamp = start_time + ros::Duration(1, 0) * (msg.header.seq / update_rate_);
//                FillROSVector3d(acc_measure, msg.linear_acceleration);
//                FillROSVector3d(gyro_measure, msg.angular_velocity);
//
//                bag_output_.write(rostopic_, msg.header.stamp, msg);
//            }
//
//            gyroscope_real = Vec3d::Zero();
//
//            for (int64_t i = update_rate_; i < update_rate_ + static_interval_time_ * update_rate_; ++i) {
//
//                // Break if requested by user
//                if (!ros::ok()) {
//                    break;
//                }
//
//                // Reference: https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
//                accelerometer_bias += RandomNormalDistributionVector(accelerometer_random_walk_) * sqrt(dt);
//                gyroscope_bias += RandomNormalDistributionVector(gyroscope_random_walk_) * sqrt(dt);
//
//                Vec3d acc_measure = accelerometer_real + accelerometer_bias +
//                                    RandomNormalDistributionVector(accelerometer_noise_density_) / sqrt(dt);
//                Vec3d gyro_measure = gyroscope_real + gyroscope_bias +
//                                     RandomNormalDistributionVector(gyroscope_noise_density_) / sqrt(dt);
//
//                sensor_msgs::Imu msg;
////                msg.header.stamp = start_time + ros::Duration(1, 0) * (i / update_rate_);
//                msg.header.seq = (init_static_period_ + interval_index * static_interval_time_) * update_rate_ + i;
//                ROS_INFO_STREAM("msg.header.seq: " << msg.header.seq);
//                msg.header.stamp = start_time + ros::Duration(1, 0) * (msg.header.seq / update_rate_);
//                FillROSVector3d(acc_measure, msg.linear_acceleration);
//                FillROSVector3d(gyro_measure, msg.angular_velocity);
//
//                bag_output_.write(rostopic_, msg.header.stamp, msg);
//            }
//        }

        // clang-format on

        ROS_INFO_STREAM("Finished generating data. ");
    }

    void GenerateStaticData() {
        ROS_INFO_STREAM("Generating IMU data ...");

        double dt = 1 / update_rate_;

        // clang-format off
        ros::Time start_time(1.0);
        Vec3d accelerometer_bias = Vec3d::Constant(accelerometer_bias_init_);
        Vec3d gyroscope_bias = Vec3d::Constant(gyroscope_bias_init_);
        Vec3d accelerometer_real = {0, 0, 9.81};
        Vec3d gyroscope_real = Vec3d::Zero();

        for (int64_t i = 0; i < init_static_period_ * update_rate_; ++i) {

            // Break if requested by user
            if (!ros::ok()) {
                break;
            }

            // Reference: https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
            accelerometer_bias += RandomNormalDistributionVector(accelerometer_random_walk_) * sqrt(dt);
            gyroscope_bias += RandomNormalDistributionVector(gyroscope_random_walk_) * sqrt(dt);

            Vec3d acc_measure = accelerometer_real + accelerometer_bias +
                                RandomNormalDistributionVector(accelerometer_noise_density_) / sqrt(dt);
            Vec3d gyro_measure = gyroscope_real + gyroscope_bias +
                                 RandomNormalDistributionVector(gyroscope_noise_density_) / sqrt(dt);

            sensor_msgs::Imu msg;
            msg.header.stamp = start_time + ros::Duration(1, 0) * (i / update_rate_);
            msg.header.seq = i;
            FillROSVector3d(acc_measure, msg.linear_acceleration);
            FillROSVector3d(gyro_measure, msg.angular_velocity);

//            FillROSVector3d(accelerometer_real, msg.linear_acceleration);
//            FillROSVector3d(gyroscope_real, msg.angular_velocity);

            bag_output_.write(rostopic_, msg.header.stamp, msg);
        }

        ROS_INFO_STREAM("Finished generating data. ");
    }

private:
    // ROS
    rosbag::Bag bag_output_;

private:
    double accelerometer_noise_density_ = 0.0025019929573561175;
    double accelerometer_random_walk_ = 6.972435158192731e-05;
    double accelerometer_bias_init_ = 0.007;

    double gyroscope_noise_density_ = 0.0001888339269965301;
    double gyroscope_random_walk_ = 2.5565313322052523e-06;
    double gyroscope_bias_init_ = 0.006;

    std::string rostopic_ = "/qcar_imu/raw";
    double update_rate_ = 2500;

    double init_static_period_ = 10800;
    double static_interval_time_ = 4;
    uint num_interval_ = 30;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "allan_variance_ros");
    ros::NodeHandle nh;
    std::string rosbag_filename;
//    std::string config_file;

    if (argc >= 2) {
        rosbag_filename = argv[1];
//        config_file = argv[2];
        ROS_INFO_STREAM("Bag filename = " << rosbag_filename);
//        ROS_INFO_STREAM("Config File = " << config_file);
    } else {
        ROS_WARN("Usage: ./imu_simulator /path/to/output/bag_filename");
        return 1;
    }

    auto start = std::clock();

    ImuSimulator simulator(rosbag_filename);
    ROS_INFO_STREAM("IMU simulator constructed");
//    simulator.GenerateDataWithIntervals();
    simulator.GenerateStaticData();

    double durationTime = (std::clock() - start) / (double) CLOCKS_PER_SEC;
    ROS_INFO("Total computation time: %f s", durationTime);
    return 0;
}