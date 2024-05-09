// Copyright (c) 2023. Created on 10/8/23 7:36 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef MI_CALIB_SIMULATION_HPP
#define MI_CALIB_SIMULATION_HPP

#include "rosbag/view.h"
#include "sensor_msgs/Imu.h"
#include "ctraj/core/trajectory.h"
#include "ctraj/core/simu_trajectory.h"
#include "calib/calib_param_manager.h"

namespace ns_mi {
    struct MultiSensorSimulator {
    protected:
        ns_ctraj::Trajectory<5>::Ptr _traj;

        const std::string _savePath;
        Eigen::Vector3d _gravity;

    public:
        explicit MultiSensorSimulator(std::string savePath) : _savePath(std::move(savePath)) {
            // trajectory
            _traj = ns_ctraj::SimuDrunkardMotion<5>({0.0, 0.0, 0.0}, 0.02, 10.0, 0.0, 10.0, 5).GetTrajectory();
            _gravity = {0.0, 0.0, -9.8};

            // visualization
            ns_ctraj::Viewer viewer;
            _traj->Visualization(viewer);
            viewer.RunInSingleThread();
        }

        void Simulate(int num, double acceNoise, double gyroNoise, double acceBias, double gyroBias) {
            auto bag = std::make_unique<rosbag::Bag>();
            bag->open(_savePath + "/simu_imus.bag", rosbag::BagMode::Write);

            auto calibParam = CalibParamManager::Create();
            calibParam->GRAVITY = _gravity;
            auto poses = GenerateUniformPoseOnSphere(num, 2.0);
            for (int i = 0; i < static_cast<int>(poses.size()); ++i) {
                auto topic = "/imu" + std::to_string(i) + "/frame";
                const auto &bias = poses.at(i);
                spdlog::info("simulate '{}'...", topic);

                auto timeOffset = i * 0.001;

                SimulateIMU(
                        bag, topic, bias, _gravity, 400, timeOffset, acceNoise, gyroNoise, acceBias, gyroBias
                );

                // record
                calibParam->EXTRI.SO3_BiToBr.insert({topic, bias.so3()});
                calibParam->EXTRI.POS_BiInBr.insert({topic, bias.translation()});
                calibParam->TEMPORAL.TIME_OFFSET_BiToBr.insert({topic, timeOffset});
                calibParam->INTRI.IMU.insert({topic, {}});
                calibParam->INTRI.IMU.at(topic).Clear();
            }
            bag->close();

            auto alignedParam = calibParam->AlignParamToNewSensor(
                    calibParam->EXTRI.SE3_BiToBr("/imu0/frame").inverse(),
                    -calibParam->TEMPORAL.TIME_OFFSET_BiToBr.at("/imu0/frame")
            );
            alignedParam->Save(_savePath + "/truth_align_to_imu0.yaml", CerealArchiveType::Enum::YAML);
            alignedParam->ShowParamStatus();

            ns_viewer::Viewer viewer;
            calibParam->VisualizationSensors(viewer);
            viewer.RunInSingleThread();
        }

        void SimulateIMU(const std::unique_ptr<rosbag::Bag> &bag, const std::string &imuTopic,
                         const Sophus::SE3d &SE3_BiToRef, const Eigen::Vector3d &gravityInRef0,
                         int hz, double timeOffset, double acceNoise, double gyroNoise, double acceBias,
                         double gyroBias) {
            std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
            std::normal_distribution<double> aNoise(0.0, acceNoise), gNoise(0.0, gyroNoise);

            // imu data
            auto imuMes = this->_traj->ComputeIMUMeasurement(
                    gravityInRef0, SE3_BiToRef, 1.0 / static_cast<double>(hz),
                    _traj->MinTime() + 1.0, _traj->MaxTime() - 1.0
            );
            // write imu msgs
            spdlog::info("insert imu messages for topic '{}', hz: '{}'", imuTopic, hz);
            for (const auto &frame: imuMes) {
                if (frame->GetTimestamp() - timeOffset < ros::TIME_MIN.toSec()) { continue; }

                sensor_msgs::Imu imuMsg;
                imuMsg.header.stamp = ros::Time(frame->GetTimestamp() - timeOffset);
                imuMsg.header.frame_id = "imu";

                imuMsg.angular_velocity.x = frame->GetGyro()(0) + gNoise(engine) + gyroBias;
                imuMsg.angular_velocity.y = frame->GetGyro()(1) + gNoise(engine) + gyroBias;
                imuMsg.angular_velocity.z = frame->GetGyro()(2) + gNoise(engine) + gyroBias;

                imuMsg.linear_acceleration.x = frame->GetAcce()(0) + aNoise(engine) + acceBias;
                imuMsg.linear_acceleration.y = frame->GetAcce()(1) + aNoise(engine) + acceBias;
                imuMsg.linear_acceleration.z = frame->GetAcce()(2) + aNoise(engine) + acceBias;

                bag->write(imuTopic, imuMsg.header.stamp, imuMsg);
            }
        }

    protected:
        static std::vector<Sophus::SE3d> GenerateUniformPoseOnSphere(int n, double r) {
            const double phi = (std::sqrt(5.0) - 1.0) * 0.5;
            std::vector<Sophus::SE3d> poses;
            for (int i = 1; i < n + 1; ++i) {
                auto z = (2.0 * i - 1.0) / n - 1;
                auto x = std::sqrt(1.0 - z * z) * std::cos(2.0 * M_PI * i * phi);
                auto y = std::sqrt(1.0 - z * z) * std::sin(2.0 * M_PI * i * phi);
                Eigen::Vector3d POS_BiInRef(x * r, y * r, z * r);
                Eigen::Vector3d zAxis = -POS_BiInRef.normalized();
                Eigen::Vector3d xAxis = TangentBasis(zAxis).block<3, 1>(0, 0);
                Eigen::Vector3d yAxis = zAxis.cross(xAxis);
                Eigen::Matrix3d rotMat;
                rotMat.col(0) = xAxis;
                rotMat.col(1) = yAxis;
                rotMat.col(2) = zAxis;
                poses.emplace_back(Sophus::SO3d(rotMat), POS_BiInRef);
            }
            return poses;
        }

        static Eigen::MatrixXd TangentBasis(const Eigen::Vector3d &g0) {
            Eigen::Vector3d b, c;
            Eigen::Vector3d a = g0.normalized();
            Eigen::Vector3d tmp(0, 0, 1);
            if (a == tmp)
                tmp << 1, 0, 0;
            b = (tmp - a * (a.transpose() * tmp)).normalized();
            c = a.cross(b);
            Eigen::MatrixXd bc(3, 2);
            bc.block<3, 1>(0, 0) = b;
            bc.block<3, 1>(0, 1) = c;
            return bc;
        }
    };
}

#endif //MI_CALIB_SIMULATION_HPP
