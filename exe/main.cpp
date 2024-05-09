// Copyright (c) 2023. Created on 10/6/23 12:17 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include "ros/ros.h"
#include "spdlog/spdlog.h"
#include "calib/calib_solver.h"

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
    ros::init(argc, argv, "mi_calib_prog_node");
    try {
        ConfigSpdlog();

        PrintLibInfo();

        // auto configor = ns_mi::Configor();
        // ns_mi::Configor::DataStream::IMUTopics.insert({"A", "B"});
        // configor.SaveConfigure("/home/csl/ros_ws/MI-Calib/src/mi_calib/config/config.yaml");
        // std::cin.get();

        // load settings
        std::string configPath;
        if (!ros::param::get("/mi_calib_prog_node/config_path", configPath)) {
            throw ns_mi::Status(
                    ns_mi::Status::Flag::CRITICAL,
                    "the configure path couldn't obtained from ros param '/mi_calib_prog_node/config_path'."
            );
        }
        spdlog::info("loading configure from json file '{}'...", configPath);

        if (!ns_mi::Configor::LoadConfigure(configPath)) {
            throw ns_mi::Status(ns_mi::Status::Flag::CRITICAL, "load configure file failed!");
        } else {
            ns_mi::Configor::PrintMainFields();
        }

        // create parameter manager
        auto paramManager = ns_mi::CalibParamManager::Create();
        // initialize and show parameter manager
        paramManager->InitializeParametersFromConfigor();
        paramManager->ShowParamStatus();
        // create data manager
        auto dataManager = ns_mi::CalibDataManager::Create();

        // pass parameter manager and data manager to solver for solving
        auto solver = ns_mi::CalibSolver::Create(dataManager, paramManager);
        solver->Process();

        // solve finished, save calibration results (file type: JSON | YAML | XML | BINARY)
        paramManager->Save(
                ns_mi::Configor::DataStream::OutputPath + "/param" + ns_mi::Configor::GetFormatExtension(),
                ns_mi::Configor::Preference::OutputDataFormat
        );

        // visualization
        // solver->VisualizationSensorSuite();
        // solver->VisualizationBSplines(ns_mi::BSplinesType::ROT_SPLINE);
        // solver->VisualizationBSplines(ns_mi::BSplinesType::LIN_ACCE_SPLINE);

        // save splines
        if (ns_mi::Configor::Preference::OutputBSplines) {
            solver->SaveBSplines();
        }
        // save kinematics
        if (ns_mi::Configor::Preference::OutputKinematics) {
            solver->SaveIMUMesEst();
        }

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
}