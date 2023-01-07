//
// Created by qin on 12/15/22.
//

//#include <vector>
#include <boost/filesystem.hpp>
#include <fstream>
//#include <set>
#include <glog/logging.h>
#include <ros/node_handle.h>
#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>

#include "base.h"
#include "calibration.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "allan_variance_ros");
    ros::NodeHandle nodeHandle;
    std::string bag_file_dir;
    std::string imu_topic = "/qcar_imu/raw";

    // Initialize Google's logging library.
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true; // log to stderr instead of logfiles

    if (argc >= 2) {
        bag_file_dir = argv[1];
        LOG(INFO) << "Bag Folder = " << bag_file_dir;
    } else {
        LOG(ERROR) << "Rosbag folder not provided!";
    }

    namespace fs = boost::filesystem;
    fs::path path = fs::absolute(fs::path(bag_file_dir));

    std::vector<imu_tk::TriadData> acc_data, gyro_data;
//    imu_tk::TimestampUnit unit = imu_tk::TIMESTAMP_UNIT_SEC;

    try {
        rosbag::Bag bag;
        bag.open(bag_file_dir, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery(imu_topic));


        // Check to make sure we have data to play
        if (view.size() == 0) {
            LOG(ERROR) << "Unable to parse any messages...";
            return 0;
        }
        LOG(INFO) << "Bag has " << view.size() << " messages, parsing...";

        // Loop through data
        time_t start = clock();
        int imu_counter = 0;
        bool first_msg = true;
        uint64_t first_msg_time, last_msg_time, tCurrNanoSeconds;
        for (const rosbag::MessageInstance &msg: view) {
            // Fill IMU buffer
            if (msg.isType<sensor_msgs::Imu>()) {
                sensor_msgs::ImuConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
                tCurrNanoSeconds = imu_msg->header.stamp.toNSec();
//                LOG(INFO) << "tCurrNanoSeconds = " << tCurrNanoSeconds;

                imu_counter++;

                // print progress
                if (difftime(clock(), start) / CLOCKS_PER_SEC >= 2.0) {
                    LOG(INFO) << imu_counter << " samples loaded";
                    start = clock();
                }

                if (first_msg) {
                    first_msg = false;
                    first_msg_time = tCurrNanoSeconds;
                    last_msg_time = tCurrNanoSeconds;
                }

                imu_tk::TriadData acc_input(imu_msg->header.stamp.toSec(), imu_msg->linear_acceleration.x,
                                                       imu_msg->linear_acceleration.y,
                                                       imu_msg->linear_acceleration.z),
                        gyro_input(imu_msg->header.stamp.toSec(), imu_msg->angular_velocity.x,
                                   imu_msg->angular_velocity.y,
                                   imu_msg->angular_velocity.z);
                acc_data.push_back(acc_input);
                gyro_data.push_back(gyro_input);
            }
            if (!nodeHandle.ok()) {
                LOG(ERROR) << "Stop requested, closing the bag!";
                bag.close();
                return 1;
            }
        }
        bag.close();

    } catch (rosbag::BagIOException &e) {
        LOG(WARNING) << "Captured rosbag::BagIOException " << e.what();
    } catch (rosbag::BagUnindexedException &e) {
        LOG(WARNING) << "Captured rosbag::BagUnindexedException " << e.what();
    } catch (std::exception &e) {
        LOG(ERROR) << e.what();
    } catch (...) {
        LOG(ERROR) << "Captured unknown exception";
    }

    LOG(INFO) << "Finished collecting " << acc_data.size() << " measurements";

    imu_tk::CalibratedTriad init_acc_calib, init_gyro_calib;
    init_acc_calib.setBias(Eigen::Vector3d(0.04222, 0.03844, 0.05533)); // why these values? units? TODO
    init_gyro_calib.setScale(Eigen::Vector3d(1.0 / 6258.0, 1.0 / 6258.0, 1.0 / 6258.0));

    imu_tk::MultiPosCalibration mp_calib;

    mp_calib.setInitStaticIntervalDuration(50.0);
    mp_calib.setInitAccCalibration(init_acc_calib);
    mp_calib.setInitGyroCalibration(init_gyro_calib);
    mp_calib.setGravityMagnitude(9.80616);
    mp_calib.enableVerboseOutput(true);
    mp_calib.enableAccUseMeans(false);
    //mp_calib.setGyroDataPeriod(0.01);
    mp_calib.calibrateAccGyro(acc_data, gyro_data);
    mp_calib.getAccCalib().save("test_imu_acc.calib");
    mp_calib.getGyroCalib().save("test_imu_gyro.calib");

    return 0;
}