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
