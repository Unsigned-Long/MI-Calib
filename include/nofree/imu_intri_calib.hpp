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

#ifndef MI_CALIB_IMU_INTRI_CALIB_HPP
#define MI_CALIB_IMU_INTRI_CALIB_HPP

#include "rosbag/view.h"
#include "sensor/imu_intrinsic.hpp"
#include "sensor/imu_data_loader.h"
#include "cereal/cereal.hpp"
#include "cereal/types/utility.hpp"
#include "nofree/imu_intri_calib_factors.hpp"
#include "ctraj/core/trajectory_estimator.h"
#include "calib/estimator.h"

namespace ns_mi {
    class IMUIntriCalibSolver {
    public:
        using Ptr = std::shared_ptr<IMUIntriCalibSolver>;

    public:
        struct Configor {
        public:
            struct Item {
                std::string bagPath;
                std::vector<std::pair<double, double>> staticPieces;

                Item() = default;

            public:
                template<class Archive>
                void serialize(Archive &ar) {
                    ar(cereal::make_nvp("BagPath", bagPath),
                       cereal::make_nvp("StaticPieces", staticPieces));
                }
            };

        public:

            std::vector<Item> items;
            std::string IMUType;
            std::string IMUTopic;
            double gravityNorm{};
            std::string outputPath;

        public:
            Configor() = default;

            // load configure information from file
            template<class CerealArchiveType=CerealArchiveType::YAML>
            static Configor LoadConfigure(const std::string &filename) {
                std::ifstream file(filename);
                auto archive = GetInputArchive<CerealArchiveType>(file);
                auto configor = Configor();
                (*archive)(cereal::make_nvp("Configor", configor));
                return configor;
            }

            // save configure information to file
            template<class CerealArchiveType=CerealArchiveType::YAML>
            bool SaveConfigure(const std::string &filename) {
                std::ofstream file(filename);
                auto archive = GetOutputArchive<CerealArchiveType>(file);
                (*archive)(cereal::make_nvp("Configor", *this));
                return true;
            }

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(cereal::make_nvp("IMUTopic", IMUTopic), CEREAL_NVP(IMUType),
                   cereal::make_nvp("GravityNorm", gravityNorm), cereal::make_nvp("OutputPath", outputPath),
                   cereal::make_nvp("ROSBags", items));
            }
        };

    protected:
        Configor configor;
        std::vector<std::list<IMUFrame::Ptr>> data;
        std::vector<Eigen::Vector3d> gravity;
        IMUIntrinsics intrinsics;

    public:
        explicit IMUIntriCalibSolver(Configor configor) : configor(std::move(configor)) {
            this->LoadIMUData();
        }

        static Ptr Create(const Configor &configor) {
            return std::make_shared<IMUIntriCalibSolver>(configor);
        }

        void Process() {
            // initialize gravity roughly
            gravity.resize(data.size());
            for (int i = 0; i < static_cast<int>(data.size()); ++i) {
                if (data.at(i).empty()) { continue; }
                gravity.at(i) = -configor.gravityNorm * AverageAcce(data.at(i)).normalized();
            }

            auto *GRAVITY_MANIFOLD = new ceres::SphereManifold<3>();
            auto *QUATER_MANIFOLD = new ceres::EigenQuaternionManifold();

            ceres::Problem prob;
            // add factors
            for (int i = 0; i < static_cast<int>(data.size()); ++i) {
                const auto &pieces = data.at(i);
                Eigen::Vector3d &grav = gravity.at(i);
                for (const auto &frame: pieces) {

                    // accelerator
                    auto acceCostFunc = IMUIntriAcceFactor::Create(frame, 1.0);
                    acceCostFunc->AddParameterBlock(3);
                    acceCostFunc->AddParameterBlock(6);
                    acceCostFunc->AddParameterBlock(3);
                    acceCostFunc->SetNumResiduals(3);

                    std::vector<double *> acceParBlockVec;
                    acceParBlockVec.push_back(intrinsics.ACCE.BIAS.data());
                    acceParBlockVec.push_back(intrinsics.ACCE.MAP_COEFF.data());
                    acceParBlockVec.push_back(grav.data());

                    prob.AddResidualBlock(acceCostFunc, nullptr, acceParBlockVec);
                    prob.SetManifold(grav.data(), GRAVITY_MANIFOLD);

                    // gyroscope
                    auto gyroCostFunc = IMUIntriGyroFactor::Create(frame, 1.0);
                    gyroCostFunc->AddParameterBlock(3);
                    gyroCostFunc->AddParameterBlock(6);
                    gyroCostFunc->AddParameterBlock(4);
                    gyroCostFunc->SetNumResiduals(3);

                    std::vector<double *> gyroParBlockVec;
                    gyroParBlockVec.push_back(intrinsics.GYRO.BIAS.data());
                    gyroParBlockVec.push_back(intrinsics.GYRO.MAP_COEFF.data());
                    gyroParBlockVec.push_back(intrinsics.SO3_AtoG.data());

                    prob.AddResidualBlock(gyroCostFunc, nullptr, gyroParBlockVec);
                    prob.SetManifold(intrinsics.SO3_AtoG.data(), QUATER_MANIFOLD);

                    // we do not optimize these two block
                    prob.SetParameterBlockConstant(intrinsics.GYRO.MAP_COEFF.data());
                    prob.SetParameterBlockConstant(intrinsics.SO3_AtoG.data());
                }
            }
            ceres::Solver::Summary summary;
            ceres::Solve(Estimator::DefaultSolverOptions(), &prob, &summary);
            spdlog::info("here is the summary:\n{}\n", summary.BriefReport());
        }

        [[nodiscard]] const IMUIntrinsics &GetIntrinsics() const {
            return intrinsics;
        }

    protected:
        void LoadIMUData() {
            auto dataLoader = IMUDataLoader::GetLoader(configor.IMUType);
            data.resize(configor.items.size());
            for (int i = 0; i < static_cast<int>(configor.items.size()); ++i) {
                const auto &item = configor.items.at(i);
                if (!std::filesystem::exists(item.bagPath)) {
                    throw Status(Status::Flag::CRITICAL, "the bag path not exists: " + item.bagPath);
                }
                auto &curData = data.at(i);
                spdlog::info("load imu data from '{}'...", item.bagPath);

                // open the ros bag
                auto bag = std::make_unique<rosbag::Bag>();
                bag->open(item.bagPath, rosbag::BagMode::Read);
                auto viewTemp = rosbag::View();
                viewTemp.addQuery(*bag, rosbag::TopicQuery(configor.IMUTopic));
                auto begTime = viewTemp.getBeginTime();
                auto endTime = viewTemp.getEndTime();

                for (const auto &[st, et]: item.staticPieces) {
                    spdlog::info("load imu data in time piece [{}, {}]...", st, et);
                    ros::Time curBegTime, curEndTime;
                    if (st < 0.0 || et < 0.0) {
                        spdlog::warn("negative time piece, using all data piece for calibration!!!");
                        curBegTime = begTime;
                        curEndTime = endTime;
                    } else {
                        curBegTime = begTime + ros::Duration(st);
                        curEndTime = begTime + ros::Duration(et);
                    }
                    if (curBegTime > endTime || curEndTime > endTime) {
                        throw Status(Status::Flag::CRITICAL, "the static pieces are not in range!");
                    }
                    auto view = rosbag::View();
                    view.addQuery(*bag, rosbag::TopicQuery(configor.IMUTopic), curBegTime, curEndTime);
                    for (const auto &frame: view) {
                        // is an inertial frame
                        auto mes = dataLoader->UnpackFrame(frame);
                        curData.push_back(mes);
                    }
                }
            }
        }

        static Eigen::Vector3d AverageAcce(const std::list<IMUFrame::Ptr> &frames) {
            Eigen::Vector3d sum = Eigen::Vector3d::Zero();
            for (const auto &item: frames) {
                sum += item->GetAcce();
            }
            return sum / frames.size();
        }
    };
}

#endif //MI_CALIB_IMU_INTRI_CALIB_HPP
