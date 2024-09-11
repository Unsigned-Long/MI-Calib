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

#ifndef MI_CALIB_CALIB_DATA_MANAGER_H
#define MI_CALIB_CALIB_DATA_MANAGER_H

#include "config/configor.h"
#include "sensor/imu.h"
#include "calib/status.hpp"

namespace ns_mi {

    class CalibDataManager {
    public:
        using Ptr = std::shared_ptr<CalibDataManager>;

    private:
        std::map <std::string, std::vector<IMUFrame::Ptr>> _imuMes;

        double _rawStartTimestamp{};
        double _rawEndTimestamp{};

        double _alignedStartTimestamp{};
        double _alignedEndTimestamp{};

    public:
        // using config information to load and adjust data in this constructor
        CalibDataManager();

        // the creator
        static CalibDataManager::Ptr Create();

        // get raw imu frames
        [[nodiscard]] const std::map <std::string, std::vector<IMUFrame::Ptr>> &
        GetIMUMeasurements() const;

        [[nodiscard]] const std::vector <IMUFrame::Ptr> &
        GetIMUMeasurements(const std::string &imuTopic) const;

        [[nodiscard]] double GetRawStartTimestamp() const;

        [[nodiscard]] double GetRawEndTimestamp() const;

        [[nodiscard]] double GetAlignedStartTimestamp() const;

        [[nodiscard]] double GetAlignedEndTimestamp() const;

        [[nodiscard]] double GetCalibStartTimestamp() const;

        [[nodiscard]] double GetCalibEndTimestamp() const;

        bool SaveIMUMeasurements(const std::string &filename, CerealArchiveType::Enum archiveType);

        [[nodiscard]] std::vector <std::vector<IMUFrame::Ptr>>
        SplitIMUMesToWins(const std::string &imuTopic, double winSize, double sTime, double eTime) const;

        static auto ExtractIMUDataPiece(const std::vector <IMUFrame::Ptr> &data, double st, double et) {
            auto sIter = std::find_if(data.begin(), data.end(), [st](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() > st;
            });
            auto eIter = std::find_if(data.rbegin(), data.rend(), [et](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() < et;
            }).base();
            return std::pair(sIter, eIter);
        }

        auto ExtractIMUDataPiece(const std::string &topic, double st, double et) {
            return ExtractIMUDataPiece(_imuMes.at(topic), st, et);
        }

    protected:
        // load camera, lidar, imu data from the ros bag [according to the config file]
        void LoadCalibData();

        // make sure the first imu frame is before camera and lidar data
        // assign the '_alignedStartTimestamp' and '_alignedEndTimestamp'
        void AdjustCalibDataSequence();

        // align the timestamp to zero
        void AlignTimestamp();

        // remove the head data according to the pred
        template<typename ElemType, typename Pred>
        void EraseSeqHeadData(std::vector <ElemType> &seq, Pred pred, const std::string &errorMsg) {
            auto iter = std::find_if(seq.begin(), seq.end(), pred);
            if (iter == seq.end()) {
                // find failed
                throw Status(Status::Flag::ERROR, errorMsg);
            } else {
                // adjust
                seq.erase(seq.begin(), iter);
            }
        }

        // remove the tail data according to the pred
        template<typename ElemType, typename Pred>
        void EraseSeqTailData(std::vector <ElemType> &seq, Pred pred, const std::string &errorMsg) {
            auto iter = std::find_if(seq.rbegin(), seq.rend(), pred);
            if (iter == seq.rend()) {
                // find failed
                throw Status(Status::Flag::ERROR, errorMsg);
            } else {
                // adjust
                seq.erase(iter.base(), seq.end());
            }
        }

        // output the data status
        void OutputDataStatus() const;
    };

}


#endif //MI_CALIB_CALIB_DATA_MANAGER_H
