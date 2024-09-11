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

#include "calib/calib_data_manager.h"
#include "rosbag/view.h"
#include "spdlog/spdlog.h"
#include "sensor_msgs/Imu.h"
#include "sensor/imu_data_loader.h"

namespace ns_mi {

    CalibDataManager::CalibDataManager() {
        LoadCalibData();
        AdjustCalibDataSequence();
        AlignTimestamp();
    }

    CalibDataManager::Ptr CalibDataManager::Create() {
        return std::make_shared<CalibDataManager>();
    }

    void CalibDataManager::LoadCalibData() {
        spdlog::info("load calibration data...");

        // open the ros bag
        auto bag = std::make_unique<rosbag::Bag>();
        if (!std::filesystem::exists(Configor::DataStream::BagPath)) {
            spdlog::error("the ros bag path '{}' is invalid!", Configor::DataStream::BagPath);
        } else {
            bag->open(Configor::DataStream::BagPath, rosbag::BagMode::Read);
        }

        auto view = rosbag::View();

        // using a temp view to check the time range of the source ros bag
        auto viewTemp = rosbag::View();

        std::vector<std::string> topicsToQuery;
        // add topics to vector
        auto IMUTopics = Configor::DataStream::IMUTopics;
        for (const auto &[item, _]: IMUTopics) { topicsToQuery.push_back(item); }

        viewTemp.addQuery(*bag, rosbag::TopicQuery(topicsToQuery));
        auto begTime = viewTemp.getBeginTime();
        auto endTime = viewTemp.getEndTime();
        spdlog::info("source data duration: from '{:.5f}' to '{:.5f}'.", begTime.toSec(), endTime.toSec());

        // adjust the data time range
        if (Configor::DataStream::BeginTime > 0.0) {
            begTime += ros::Duration(Configor::DataStream::BeginTime);
            if (begTime > endTime) {
                spdlog::warn(
                        "begin time '{:.5f}' is out of the bag's data range, set begin time to '{:.5f}'.",
                        begTime.toSec(), viewTemp.getBeginTime().toSec()
                );
                begTime = viewTemp.getBeginTime();
            }
        }
        if (Configor::DataStream::Duration > 0.0) {
            endTime = begTime + ros::Duration(Configor::DataStream::Duration);
            if (endTime > viewTemp.getEndTime()) {
                spdlog::warn(
                        "end time '{:.5f}' is out of the bag's data range, set end time to '{:.5f}'.",
                        endTime.toSec(), viewTemp.getEndTime().toSec()
                );
                endTime = viewTemp.getEndTime();
            }
        }
        spdlog::info("expect data duration: from '{:.5f}' to '{:.5f}'.", begTime.toSec(), endTime.toSec());

        view.addQuery(*bag, rosbag::TopicQuery(topicsToQuery), begTime, endTime);

        // create IMU data loader
        std::map<std::string, IMUDataLoader::Ptr> imuDataLoaders;

        // get radar type enum from the string
        for (const auto &[imuTopic, config]: Configor::DataStream::IMUTopics) {
            imuDataLoaders.insert({imuTopic, IMUDataLoader::GetLoader(config.Type)});
        }

        // read raw data
        for (const auto &item: view) {
            const std::string &topic = item.getTopic();
            if (IMUTopics.cend() != IMUTopics.find(topic)) {

                auto msg = imuDataLoaders.at(topic)->UnpackFrame(item);
                _imuMes[topic].push_back(msg);

            }
        }
        bag->close();
        OutputDataStatus();
    }

    void CalibDataManager::AdjustCalibDataSequence() {
        spdlog::info("adjust calibration data sequence...");

        // make sure the first and last frame is imu frame
        auto imuMinTime = std::max_element(_imuMes.begin(), _imuMes.end(), [](const auto &p1, const auto &p2) {
            return p1.second.front()->GetTimestamp() < p2.second.front()->GetTimestamp();
        })->second.front()->GetTimestamp();
        auto imuMaxTime = std::min_element(_imuMes.begin(), _imuMes.end(), [](const auto &p1, const auto &p2) {
            return p1.second.back()->GetTimestamp() < p2.second.back()->GetTimestamp();
        })->second.back()->GetTimestamp();

        _rawStartTimestamp = imuMinTime;
        _rawEndTimestamp = imuMaxTime;

        for (const auto &[topic, _]: Configor::DataStream::IMUTopics) {
            // remove imu frames that are before the start time stamp
            EraseSeqHeadData(_imuMes.at(topic), [this](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() > _rawStartTimestamp;
            }, "the imu data is invalid, there is no intersection.");

            // remove imu frames that are after the end time stamp
            EraseSeqTailData(_imuMes.at(topic), [this](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() < _rawEndTimestamp;
            }, "the imu data is invalid, there is no intersection.");
        }

        OutputDataStatus();
    }

    void CalibDataManager::AlignTimestamp() {
        spdlog::info("align calibration data timestamp...");

        _alignedStartTimestamp = 0.0;
        _alignedEndTimestamp = _rawEndTimestamp - _rawStartTimestamp;
        for (auto &[imuTopic, mes]: _imuMes) {
            for (const auto &frame: mes) {
                frame->SetTimestamp(frame->GetTimestamp() - _rawStartTimestamp);
            }
        }

        OutputDataStatus();
    }

    void CalibDataManager::OutputDataStatus() const {
        spdlog::info("calibration data info:");
        for (const auto &[imuTopic, mes]: _imuMes) {
            spdlog::info("IMU topic: '{}', data size: '{:06}', time span: from '{:+010.5f}' to '{:+010.5f}' (s)",
                         imuTopic, mes.size(), mes.front()->GetTimestamp(), mes.back()->GetTimestamp());
        }

        spdlog::info("raw start time: '{:+010.5f}' (s), raw end time: '{:+010.5f}' (s)",
                     _rawStartTimestamp, _rawEndTimestamp);
        spdlog::info("aligned start time: '{:+010.5f}' (s), aligned end time: '{:+010.5f}' (s)\n",
                     _alignedStartTimestamp, _alignedEndTimestamp);
    }

    double CalibDataManager::GetRawStartTimestamp() const {
        return _rawStartTimestamp;
    }

    double CalibDataManager::GetRawEndTimestamp() const {
        return _rawEndTimestamp;
    }

    double CalibDataManager::GetAlignedStartTimestamp() const {
        return _alignedStartTimestamp;
    }

    double CalibDataManager::GetAlignedEndTimestamp() const {
        return _alignedEndTimestamp;
    }

    const std::map<std::string, std::vector<IMUFrame::Ptr>> &CalibDataManager::GetIMUMeasurements() const {
        return _imuMes;
    }

    const std::vector<IMUFrame::Ptr> &CalibDataManager::GetIMUMeasurements(const std::string &imuTopic) const {
        return _imuMes.at(imuTopic);
    }

    bool CalibDataManager::SaveIMUMeasurements(const std::string &filename, CerealArchiveType::Enum archiveType) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        auto archive = GetOutputArchiveVariant(file, archiveType);
        SerializeByOutputArchiveVariant(archive, archiveType, cereal::make_nvp("imu_mes", this->_imuMes));
        return true;
    }

    std::vector<std::vector<IMUFrame::Ptr>>
    CalibDataManager::SplitIMUMesToWins(const std::string &imuTopic, double winSize, double sTime, double eTime) const {
        std::optional<double> winEndTime = {};
        std::vector<IMUFrame::Ptr> win;
        std::vector<std::vector<IMUFrame::Ptr>> wins;
        for (const auto &frame: GetIMUMeasurements().at(imuTopic)) {
            // use the data whose timestamp is valid
            if (frame->GetTimestamp() < sTime || frame->GetTimestamp() > eTime) {
                continue;
            }
            if (winEndTime == std::nullopt) {
                winEndTime = frame->GetTimestamp() + winSize;
            }
            if (frame->GetTimestamp() < *winEndTime) {
                win.push_back(frame);
            } else {
                wins.push_back(win);
                win.clear();
                win.push_back(frame);
                winEndTime = frame->GetTimestamp() + winSize;
            }
        }
        return wins;
    }

    double CalibDataManager::GetCalibStartTimestamp() const {
        return _alignedStartTimestamp + Configor::Prior::TimeOffsetPadding;
    }

    double CalibDataManager::GetCalibEndTimestamp() const {
        return _alignedEndTimestamp - Configor::Prior::TimeOffsetPadding;
    }
}