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

#include "ros/ros.h"
#include "calib/status.hpp"
#include "spdlog/spdlog.h"
#include "nofree/simulation.hpp"
#include "nofree/vel_integration.hpp"
#include "nofree/error_statistics.hpp"

// config the 'spdlog' log pattern
void ConfigSpdlog() {
    // [log type]-[thread]-[time] message
    spdlog::set_pattern("%^[%L]%$-[%t]-[%H:%M:%S.%e] %v");

    // set log level
    spdlog::set_level(spdlog::level::debug);
}

void PrintLibInfo() {
    // ns_pretab::PrettyTable tab;
    // tab.addRowGrids(0, 1, 0, 2, ns_pretab::Align::CENTER, "");
    // tab.addGrid(1, 0, "MI-Calib");
    // tab.addGrid(1, 1, "https://github.com/Unsigned-Long/MI-Calib.git");
    // tab.addGrid(2, 0, "Author");
    // tab.addGrid(2, 1, "Shuolong Chen");
    // tab.addGrid(3, 0, "E-Mail");
    // tab.addGrid(3, 1, "shlchen@whu.edu.cn");
    // std::cout << tab << std::endl;
    std::cout << "+--------------------------------------------------------------------+\n"
                 "| _|      _|  _|_|_|              _|_|_|            _|  _|  _|       |\n"
                 "| _|_|  _|_|    _|              _|          _|_|_|  _|      _|_|_|   |\n"
                 "| _|  _|  _|    _|  _|_|_|_|_|  _|        _|    _|  _|  _|  _|    _| |\n"
                 "| _|      _|    _|              _|        _|    _|  _|  _|  _|    _| |\n"
                 "| _|      _|  _|_|_|              _|_|_|    _|_|_|  _|  _|  _|_|_|   |\n"
                 "+-----------+--------------------------------------------------------+\n"
                 "| RIs-Calib | https://github.com/Unsigned-Long/MI-Calib.git          |\n"
                 "+-----------+--------------------------------------------------------+\n"
                 "| Author    | Shuolong Chen                                          |\n"
                 "+-----------+--------------------------------------------------------+\n"
                 "| E-Mail    | shlchen@whu.edu.cn                                     |\n"
                 "+-----------+--------------------------------------------------------+" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mi_calib_learn_node");
    try {
        ConfigSpdlog();

        PrintLibInfo();

        // ns_mi::VelIntegrationLearner::Learn();

        // ns_mi::ErrorStatistic::SaveSTParamsAsRawJson(ns_mi::ErrorStatistic::PerformForParamsIter(
        //         "/home/csl/ros_ws/MI-Calib/src/mi_calib/output/simu/params_iter",
        //         "/home/csl/ros_ws/MI-Calib/src/mi_calib/output/simu/truth_align_to_imu0.yaml", "/imu0/frame"
        // ), "/home/csl/paper_create/MI-Calib-Paper/manuscript/img/simulation/opt_process/errors.json");

        ns_mi::ErrorStatistic::SaveSTParamsAsRawJsonV2(ns_mi::ErrorStatistic::PerformForParamsIter(
                "/home/csl/ros_ws/MI-Calib/src/mi_calib/output/real-world/data_202392616822/params_iter",
                "/home/csl/ros_ws/MI-Calib/src/mi_calib/output/real-world/identity.yaml", "/imu1/frame"
        ), "/home/csl/paper_create/MI-Calib-Paper/manuscript/img/real-world/opt_process/data/data_202392616822.json");

        ns_mi::ErrorStatistic::SaveSTParamsAsRawJsonV2(ns_mi::ErrorStatistic::PerformForParamsIter(
                "/home/csl/ros_ws/MI-Calib/src/mi_calib/output/real-world/data_2023926145954/params_iter",
                "/home/csl/ros_ws/MI-Calib/src/mi_calib/output/real-world/identity.yaml", "/imu1/frame"
        ), "/home/csl/paper_create/MI-Calib-Paper/manuscript/img/real-world/opt_process/data/data_2023926145954.json");

        ns_mi::ErrorStatistic::SaveSTParamsAsRawJsonV2(ns_mi::ErrorStatistic::PerformForParamsIter(
                "/home/csl/ros_ws/MI-Calib/src/mi_calib/output/real-world/data_2023926155612/params_iter",
                "/home/csl/ros_ws/MI-Calib/src/mi_calib/output/real-world/identity.yaml", "/imu1/frame"
        ), "/home/csl/paper_create/MI-Calib-Paper/manuscript/img/real-world/opt_process/data/data_2023926155612.json");

    } catch (const ns_mi::Status &status) {
        // if error happened, print it
        switch (status.flag) {
            case ns_mi::Status::Flag::FINE:
                // this case usually won't happen
                spdlog::info(status.what);
                break;
            case ns_mi::Status::Flag::WARNING:
                spdlog::warn(status.what);
                break;
            case ns_mi::Status::Flag::ERROR:
                spdlog::error(status.what);
                break;
            case ns_mi::Status::Flag::CRITICAL:
                spdlog::critical(status.what);
                break;
        }
    } catch (const std::exception &e) {
        // an unknown exception not thrown by this program
        spdlog::critical(e.what());
    }

    ros::shutdown();
    return 0;
    return 0;
}