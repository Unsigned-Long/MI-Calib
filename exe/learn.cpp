// Copyright (c) 2023. Created on 10/6/23 12:19 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

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