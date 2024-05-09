// Copyright (c) 2023. Created on 7/25/23 1:33 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef MI_CALIB_ERROR_STATISTICS_HPP
#define MI_CALIB_ERROR_STATISTICS_HPP

#include "calib/calib_param_manager.h"
#include "util/utils.hpp"

namespace ns_mi {
    struct ErrorStatistic {
        static CalibParamManager::Ptr
        Perform(const CalibParamManager::Ptr &srcParam, const CalibParamManager::Ptr &tarParam) {
            auto resParam = CalibParamManager::Create();

            // extrinsic
            for (const auto &[topic, SO3_BiToBr]: srcParam->EXTRI.SO3_BiToBr) {
                auto error = tarParam->EXTRI.SO3_BiToBr.at(topic).inverse() * SO3_BiToBr;
                resParam->EXTRI.SO3_BiToBr[topic] = error;
            }
            for (const auto &[topic, POS_BiInBr]: srcParam->EXTRI.POS_BiInBr) {
                Eigen::Vector3d error = POS_BiInBr - tarParam->EXTRI.POS_BiInBr.at(topic);
                resParam->EXTRI.POS_BiInBr[topic] = error;
            }
            // temporal
            for (const auto &[topic, TIME_OFFSET_BiToBr]: srcParam->TEMPORAL.TIME_OFFSET_BiToBr) {
                auto error = TIME_OFFSET_BiToBr - tarParam->TEMPORAL.TIME_OFFSET_BiToBr.at(topic);
                resParam->TEMPORAL.TIME_OFFSET_BiToBr[topic] = error;
            }
            // do not consider the intrinsics
            resParam->INTRI = tarParam->INTRI;
            return resParam;
        }

        static std::vector<CalibParamManager::Ptr>
        PerformForParamsIter(const std::string &paramsIterDir, const std::string &gtParamPath,
                             const std::string &refIMUTopic) {
            // ------------------------------
            // get dst files in the directory
            // ------------------------------
            auto files = FilesInDir(paramsIterDir);
            files.erase(std::remove_if(files.begin(), files.end(), [](const std::string &str) {
                auto strVec = SplitString(SplitString(str, '/').back(), '.');
                return strVec.back() != "yaml" || strVec.front().substr(0, 5) != "param";
            }), files.end());

            // sort the tarParamFile vector
            std::sort(files.begin(), files.end(), [](const std::string &str1, const std::string &str2) {
                auto file1 = SplitString(SplitString(str1, '/').back(), '.').front();
                auto file2 = SplitString(SplitString(str2, '/').back(), '.').front();
                auto idx1 = std::stoi(SplitString(file1, '_').back());
                auto idx2 = std::stoi(SplitString(file2, '_').back());
                return idx1 < idx2;
            });
            std::vector<CalibParamManager::Ptr> params;
            auto tarParam = CalibParamManager::Load<CerealArchiveType::YAML>(gtParamPath)
                    ->AlignParamToNewSensor(refIMUTopic);
            for (const auto &filename: files) {
                spdlog::info("process param file named '{}'...", filename);
                auto srcParam = CalibParamManager::Load<CerealArchiveType::YAML>(filename)
                        ->AlignParamToNewSensor(refIMUTopic);
                params.push_back(Perform(srcParam, tarParam));
            }
            return params;
        }

        static void
        SaveSTParamsAsRawJson(const std::vector<CalibParamManager::Ptr> &params, const std::string &saveFilename) {
            std::map<std::string, std::vector<Eigen::Vector3d>> euler_error, pos_error;
            std::map<std::string, std::vector<double>> temp_error;
            for (const auto &param: params) {
                for (const auto &[topic, _]: param->EXTRI.SO3_BiToBr) {
                    euler_error[topic].push_back(param->EXTRI.EULER_BiToBr_DEG(topic));
                    pos_error[topic].push_back(param->EXTRI.POS_BiInBr.at(topic));

                    temp_error[topic].push_back(param->TEMPORAL.TIME_OFFSET_BiToBr.at(topic));
                }
            }
            std::ofstream of(saveFilename);
            cereal::JSONOutputArchive ar(of);
            ar(CEREAL_NVP(euler_error), CEREAL_NVP(pos_error), CEREAL_NVP(temp_error));
        }

        static void
        SaveSTParamsAsRawJsonV2(const std::vector<CalibParamManager::Ptr> &params, const std::string &saveFilename) {
            std::map<std::string, std::vector<Eigen::Vector4d>> quat_error;
            std::map<std::string, std::vector<Eigen::Vector3d>> pos_error;
            std::map<std::string, std::vector<double>> temp_error;
            for (const auto &param: params) {
                for (const auto &[topic, _]: param->EXTRI.SO3_BiToBr) {
                    // qx, qy, qz, qw
                    quat_error[topic].push_back(param->EXTRI.SO3_BiToBr.at(topic).unit_quaternion().coeffs());
                    pos_error[topic].push_back(param->EXTRI.POS_BiInBr.at(topic));

                    temp_error[topic].push_back(param->TEMPORAL.TIME_OFFSET_BiToBr.at(topic));
                }
            }
            std::ofstream of(saveFilename);
            cereal::JSONOutputArchive ar(of);
            ar(CEREAL_NVP(quat_error), CEREAL_NVP(pos_error), CEREAL_NVP(temp_error));
        }
    };
}

#endif //MI_CALIB_ERROR_STATISTICS_HPP
