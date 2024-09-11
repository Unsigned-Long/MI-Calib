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

#ifndef MI_CALIB_CALIB_PARAM_MANAGER_H
#define MI_CALIB_CALIB_PARAM_MANAGER_H

#include "sophus/se3.hpp"
#include "ctraj/view/traj_viewer.h"
#include "cereal/types/map.hpp"
#include "spdlog/spdlog.h"
#include "config/configor.h"

namespace ns_mi {
    struct CalibParamManager {
    public:
        using Ptr = std::shared_ptr<CalibParamManager>;

    public:
        // trans radian angle to degree angle
        constexpr static double RAD_TO_DEG = 180.0 / M_PI;
        // trans degree angle to radian angle
        constexpr static double DEG_TO_RAD = M_PI / 180.0;

        // extrinsic
        struct {
            std::map<std::string, Sophus::SO3d> SO3_BiToBr;
            std::map<std::string, Eigen::Vector3d> POS_BiInBr;

            // lie algebra vector space se3
            [[nodiscard]] Sophus::SE3d SE3_BiToBr(const std::string &imuTopic) const {
                return {SO3_BiToBr.at(imuTopic), POS_BiInBr.at(imuTopic)};
            }

            // quaternion
            [[nodiscard]] Eigen::Quaterniond Q_BiToBr(const std::string &imuTopic) const {
                return SO3_BiToBr.at(imuTopic).unit_quaternion();
            }

            // the euler angles [radian and degree format]
            [[nodiscard]] Eigen::Vector3d EULER_BiToBr_RAD(const std::string &imuTopic) const {
                return Q_BiToBr(imuTopic).toRotationMatrix().eulerAngles(0, 1, 2);
            }

            [[nodiscard]] Eigen::Vector3d EULER_BiToBr_DEG(const std::string &imuTopic) const {
                auto euler = EULER_BiToBr_RAD(imuTopic);
                for (int i = 0; i != 3; ++i) { euler(i) *= CalibParamManager::RAD_TO_DEG; }
                return euler;
            }

            std::map<const double *, std::string> GetParamAddressWithDesc() {
                std::map<const double *, std::string> infoMap;

#define SAVE_EXTRI_INFO(param) infoMap.insert(std::make_pair(param.data(), #param));

                // imu
                for (const auto &[topic, data]: SO3_BiToBr) {
                    infoMap.insert({data.data(), "SO3_BiToBr[" + topic + "]"});
                }
                for (const auto &[topic, data]: POS_BiInBr) {
                    infoMap.insert({data.data(), "POS_BiInBr[" + topic + "]"});
                }

#undef SAVE_EXTRI_INFO

                return infoMap;
            }


            std::vector<Sophus::SO3d *> SO3_BiToBr_AddressVec() {
                std::vector<Sophus::SO3d *> addressVec(SO3_BiToBr.size());
                std::transform(SO3_BiToBr.begin(), SO3_BiToBr.end(), addressVec.begin(), [](auto &p) {
                    return &(p.second);
                });
                return addressVec;
            }

            std::vector<Eigen::Vector3d *> POS_BiInBr_AddressVec() {
                std::vector<Eigen::Vector3d *> addressVec(POS_BiInBr.size());
                std::transform(POS_BiInBr.begin(), POS_BiInBr.end(), addressVec.begin(), [](auto &p) {
                    return &(p.second);
                });
                return addressVec;
            }

        public:
            // Serialization
            template<class Archive>
            void serialize(Archive &archive) {
                archive(CEREAL_NVP(SO3_BiToBr), CEREAL_NVP(POS_BiInBr));
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        } EXTRI;

        // temporal
        struct {
            std::map<std::string, double> TIME_OFFSET_BiToBr;

            std::vector<double *> TIME_OFFSET_BiToBr_AddressVec() {
                std::vector<double *> addressVec(TIME_OFFSET_BiToBr.size());
                std::transform(TIME_OFFSET_BiToBr.begin(), TIME_OFFSET_BiToBr.end(), addressVec.begin(), [](auto &p) {
                    return &(p.second);
                });
                return addressVec;
            }


            std::map<const double *, std::string> GetParamAddressWithDesc() {
                std::map<const double *, std::string> infoMap;

#define SAVE_TEMPORAL_INFO(param) infoMap.insert(std::make_pair(&param, #param));

                for (const auto &[topic, data]: TIME_OFFSET_BiToBr) {
                    infoMap.insert({&data, "TIME_OFFSET_BiToBr[" + topic + "]"});
                }

#undef SAVE_TEMPORAL_INFO

                return infoMap;
            }

        public:
            // Serialization
            template<class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(TIME_OFFSET_BiToBr));
            }
        } TEMPORAL;

        // intrinsic
        struct {

            // [ topic, param pack ]
            std::map<std::string, IMUIntrinsics> IMU;


            std::map<const double *, std::string> GetParamAddressWithDesc() {
                std::map<const double *, std::string> infoMap;
                // IMU
                for (const auto &[topic, pack]: IMU) {
                    infoMap.insert({pack.ACCE.MAP_COEFF.data(), "IMU.ACCE.MAP_COEFF[" + topic + "]"});
                    infoMap.insert({pack.ACCE.BIAS.data(), "IMU.ACCE.BIAS[" + topic + "]"});
                    infoMap.insert({pack.GYRO.MAP_COEFF.data(), "IMU.GYRO.MAP_COEFF[" + topic + "]"});
                    infoMap.insert({pack.GYRO.BIAS.data(), "IMU.GYRO.BIAS[" + topic + "]"});
                    infoMap.insert({pack.SO3_AtoG.data(), "IMU.SO3_AtoG[" + topic + "]"});
                }

                return infoMap;
            }

        public:
            // Serialization
            template<class Archive>
            void serialize(Archive &archive) {
                archive(CEREAL_NVP(IMU));
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        } INTRI;

        // S2Manifold
        Eigen::Vector3d GRAVITY;

    public:

        // the constructor
        explicit CalibParamManager();

        // the creator
        static CalibParamManager::Ptr Create();

        // save the parameters to file using cereal library
        template<class CerealArchiveType=CerealArchiveType::YAML>
        void Save(const std::string &filename) const {
            std::ofstream file(filename, std::ios::out);
            auto ar = GetOutputArchive<CerealArchiveType>(file);

            (*ar)(cereal::make_nvp("CalibParam", *this));
        }

        // load the parameters from file using cereal library
        template<class CerealArchiveType=CerealArchiveType::YAML>
        static CalibParamManager::Ptr Load(const std::string &filename) {
            auto calibParamManager = CalibParamManager::Create();
            std::ifstream file(filename, std::ios::in);
            auto ar = GetInputArchive<CerealArchiveType>(file);

            (*ar)(cereal::make_nvp("CalibParam", *calibParamManager));
            return calibParamManager;
        }

        // save the parameters to file using cereal library
        void Save(const std::string &filename, CerealArchiveType::Enum archiveType) const;

        // load the parameters from file using cereal library
        static CalibParamManager::Ptr Load(const std::string &filename, CerealArchiveType::Enum archiveType);

        // print the parameters in the console
        void ShowParamStatus();

        void VisualizationSensors(ns_viewer::Viewer &viewer) const;

        // set the params to the init values, the intrinsic coeff of camera will load from the config file
        // make sure load and check config before initialize the parameters
        void InitializeParametersFromConfigor();

        [[nodiscard]] CalibParamManager::Ptr
        AlignParamToNewSensor(const Sophus::SE3d &SE3_BcToNew, double TF_BcToNew) const;

        [[nodiscard]] CalibParamManager::Ptr AlignParamToNewSensor(const std::string &topic) const;

    public:
        // Serialization
        template<class Archive>
        void serialize(Archive &archive) {
            archive(CEREAL_NVP(EXTRI), CEREAL_NVP(TEMPORAL), CEREAL_NVP(INTRI), CEREAL_NVP(GRAVITY));
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}


#endif //MI_CALIB_CALIB_PARAM_MANAGER_H
