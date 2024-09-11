// MI-Calib: An Open-Source Spatiotemporal Calibrator for Multiple IMUs Based on Continuous-Time Batch Optimization
// Copyright 2024, the School of Geodesy and Geomatics (SGG), Wuhan University, China
// https://github.com/Unsigned-Long/MI-Calib.git
//
// Author: Shuolong Chen (shlchen@whu.edu.cn)
// GitHub: https://github.com/Unsigned-Long
//  ORCID: 0000-0002-5283-9057
//
// Purpose: See .h/.hpp file.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * The names of its contributors can not be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "sensor/imu_data_loader.h"
#include "util/enum_cast.hpp"
#include "calib/status.hpp"
#include "spdlog/fmt/fmt.h"

namespace ns_mi {

    IMUDataLoader::IMUDataLoader(IMUModelType imuModel) : _imuModel(imuModel) {}

    IMUDataLoader::Ptr IMUDataLoader::GetLoader(const std::string &imuModelStr) {
        // try extract radar model
        IMUModelType imuModel;
        try {
            imuModel = EnumCast::stringToEnum<IMUModelType>(imuModelStr);
        } catch (...) {
            throw Status(
                    Status::Flag::WARNING,
                    fmt::format(
                            "Unsupported IMU Type: '{}'. "
                            "Currently supported IMU types are: \n"
                            "(1) SENSOR_IMU: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html\n"
                            "(2)    SBG_IMU: https://github.com/SBG-Systems/sbg_ros_driver.git\n"
                            "...\n"
                            "If you need to use other IMU types, "
                            "please 'Issues' us on the profile of the github repository.",
                            imuModelStr
                    )
            );
        }
        IMUDataLoader::Ptr imuDataLoader;
        switch (imuModel) {
            case IMUModelType::SENSOR_IMU:
                imuDataLoader = SensorIMULoader::Create(imuModel);
                break;
            case IMUModelType::SBG_IMU:
                imuDataLoader = SbgIMULoader::Create(imuModel);
                break;
        }
        return imuDataLoader;
    }

    IMUModelType IMUDataLoader::GetIMUModel() const {
        return _imuModel;
    }

    SensorIMULoader::SensorIMULoader(IMUModelType imuModel) : IMUDataLoader(imuModel) {}

    SensorIMULoader::Ptr SensorIMULoader::Create(IMUModelType imuModel) {
        return std::make_shared<SensorIMULoader>(imuModel);
    }

    IMUFrame::Ptr SensorIMULoader::UnpackFrame(const rosbag::MessageInstance &msgInstance) {
        // imu data item
        sensor_msgs::ImuConstPtr msg = msgInstance.instantiate<sensor_msgs::Imu>();

        auto acce = Eigen::Vector3d(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z
        );
        auto gyro = Eigen::Vector3d(
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z
        );

        return IMUFrame::Create(msg->header.stamp.toSec(), gyro, acce);
    }

    SbgIMULoader::SbgIMULoader(IMUModelType imuModel) : IMUDataLoader(imuModel) {}

    SbgIMULoader::Ptr SbgIMULoader::Create(IMUModelType imuModel) {
        return std::make_shared<SbgIMULoader>(imuModel);
    }

    IMUFrame::Ptr SbgIMULoader::UnpackFrame(const rosbag::MessageInstance &msgInstance) {
        // imu data item
        sbg_driver::SbgImuDataConstPtr msg = msgInstance.instantiate<sbg_driver::SbgImuData>();

        auto acce = Eigen::Vector3d(
                msg->accel.x,
                msg->accel.y,
                msg->accel.z
        );
        auto gyro = Eigen::Vector3d(
                msg->gyro.x,
                msg->gyro.y,
                msg->gyro.z
        );

        return IMUFrame::Create(msg->header.stamp.toSec(), gyro, acce);
    }
}