// Copyright (c) 2023. Created on 7/7/23 1:31 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include "calib/calib_solver.h"
#include "tiny-viewer/core/multi_viewer.h"
#include "calib/estimator.h"
#include "cereal/types/list.hpp"
#include "pangolin/display/display.h"

namespace ns_mi {

    // -----------
    // CalibSolver
    // -----------
    const std::string CalibSolver::VIEW_SENSORS = "VIEW_SENSORS";
    const std::string CalibSolver::VIEW_SPLINE = "VIEW_SPLINE";

    CalibSolver::CalibSolver(CalibDataManager::Ptr calibDataManager, CalibParamManager::Ptr calibParamManager)
            : _dataMagr(std::move(calibDataManager)), _parMagr(std::move(calibParamManager)),
              _ceresOption(Estimator::DefaultSolverOptions(
                      Configor::Preference::ThreadsToUse, true, Configor::Preference::UseCudaInSolving)
              ), _solveFinished(false) {
        auto so3SplineInfo = ns_ctraj::SplineInfo(
                Configor::Preference::SO3_SPLINE, ns_ctraj::SplineType::So3Spline,
                _dataMagr->GetCalibStartTimestamp(), _dataMagr->GetCalibEndTimestamp(),
                Configor::Prior::KnotTimeDist::SO3Spline
        );
        auto acceSplineInfo = ns_ctraj::SplineInfo(
                Configor::Preference::ACCE_SPLINE, ns_ctraj::SplineType::RdSpline,
                _dataMagr->GetCalibStartTimestamp(), _dataMagr->GetCalibEndTimestamp(),
                Configor::Prior::KnotTimeDist::LinAcceSpline
        );
        _splines = SplineBundleType::Create({so3SplineInfo, acceSplineInfo});

        // view create
        ns_viewer::MultiViewerConfigor viewConfig({VIEW_SENSORS, VIEW_SPLINE}, "MI-Calib");
        viewConfig.grid.at(VIEW_SENSORS).showGrid = false;
        viewConfig.grid.at(VIEW_SENSORS).showIdentityCoord = false;
        viewConfig.WithScreenShotSaveDir(Configor::DataStream::OutputPath);
        for (const auto &name: viewConfig.subWinNames) {
            viewConfig.camera.at(name).initPos = {2.0f, 2.0f, 2.0f};
            viewConfig.grid.at(name).cellSize = 0.1;
        }
        _viewer = ns_viewer::MultiViewer::Create(viewConfig);
        _viewer->RunInMultiThread();

        _ceresOption.callbacks.push_back(new MICeresViewerCallBack(_parMagr, _viewer, _splines));
        _ceresOption.update_state_every_iteration = true;
        if (Configor::Preference::OutputParamInEachIter) {
            _ceresOption.callbacks.push_back(new MICeresDebugCallBack(_parMagr));
        }
    }

    CalibSolver::Ptr CalibSolver::Create(const CalibDataManager::Ptr &calibDataManager,
                                         const CalibParamManager::Ptr &calibParamManager) {
        return std::make_shared<CalibSolver>(calibDataManager, calibParamManager);
    }

    void CalibSolver::Process() {
        // --------------
        // initialization
        // --------------
        spdlog::info("initialization...");
        Initialization();
        _parMagr->ShowParamStatus();

        // ------------------
        // batch optimization
        // ------------------
        using Opt = OptOption::Option;
        spdlog::info("batch optimization...");

        Opt optOption = Opt::OPT_SO3_SPLINE | Opt::OPT_LIN_ACCE_SPLINE | Opt::OPT_GRAVITY |
                        Opt::OPT_SO3_BiToBr | Opt::OPT_POS_BiInBr | Opt::OPT_TIME_OFFSET_BiToBr;

        BatchOptimization(optOption);
        _parMagr->ShowParamStatus();

        _solveFinished = true;
        spdlog::info("Solving is finished! Focus on the viewer and press [ctrl+'s'] to save the current scene!");
    }

    void CalibSolver::Initialization() {
        // ----------------------------------------------------------------------------------------------------
        // Step 1: initialize (i) so3 spline of the reference IMU, (ii) extrinsic rotations, (iii) time offsets
        // ----------------------------------------------------------------------------------------------------
        spdlog::info("fit rotation b-spline...");

        // step 1.1: using the reference to recover the rough so3 spline
        auto estimator = Estimator::Create(_splines, _parMagr);
        auto optOption = OptOption::Option::OPT_SO3_SPLINE;
        // add gyroscope residuals
        for (const auto &item: _dataMagr->GetIMUMeasurements(Configor::DataStream::ReferIMU)) {
            estimator->AddIMUGyroMeasurement(
                    item, Configor::DataStream::ReferIMU, optOption, Configor::Prior::Weight::GyroWeight
            );
        }
        auto sum = estimator->Solve(_ceresOption);
        spdlog::info("here is the summary:\n{}\n", sum.BriefReport());

        // step 1.2: initialize the extrinsic rotations, time offsets
        optOption = OptOption::Option::OPT_SO3_BiToBr;
        if (Configor::Preference::OptTemporalParams) { optOption |= OptOption::Option::OPT_TIME_OFFSET_BiToBr; }
        estimator = Estimator::Create(_splines, _parMagr);
        // add gyroscope residuals
        for (const auto &[topic, mes]: _dataMagr->GetIMUMeasurements()) {
            for (const auto &item: mes) {
                estimator->AddIMUGyroMeasurement(item, topic, optOption, Configor::Prior::Weight::GyroWeight);
            }
        }
        estimator->SetParameterBlockConstant(_parMagr->EXTRI.SO3_BiToBr.at(Configor::DataStream::ReferIMU).data());
        estimator->SetParameterBlockConstant(&_parMagr->TEMPORAL.TIME_OFFSET_BiToBr.at(Configor::DataStream::ReferIMU));
        sum = estimator->Solve(_ceresOption);
        spdlog::info("here is the summary:\n{}\n", sum.BriefReport());

        // step 1.3: refine
        optOption = OptOption::Option::OPT_SO3_SPLINE | OptOption::Option::OPT_SO3_BiToBr;
        if (Configor::Preference::OptTemporalParams) { optOption |= OptOption::Option::OPT_TIME_OFFSET_BiToBr; }
        estimator = Estimator::Create(_splines, _parMagr);
        // add gyroscope residuals
        for (const auto &[topic, mes]: _dataMagr->GetIMUMeasurements()) {
            for (const auto &item: mes) {
                estimator->AddIMUGyroMeasurement(item, topic, optOption, Configor::Prior::Weight::GyroWeight);
            }
        }
        estimator->SetParameterBlockConstant(_parMagr->EXTRI.SO3_BiToBr.at(Configor::DataStream::ReferIMU).data());
        estimator->SetParameterBlockConstant(&_parMagr->TEMPORAL.TIME_OFFSET_BiToBr.at(Configor::DataStream::ReferIMU));
        sum = estimator->Solve(_ceresOption);
        spdlog::info("here is the summary:\n{}\n", sum.BriefReport());

        // -----------------------------------------------------------
        // Step 2: initialize (i) gravity, (ii) extrinsic translations
        // -----------------------------------------------------------
        spdlog::info("performing gravity initialization...");
        estimator = Estimator::Create(_splines, _parMagr);
        optOption = OptOption::Option::OPT_POS_BiInBr | OptOption::Option::OPT_GRAVITY;

        const auto &so3Spline = _splines->GetSo3Spline(Configor::Preference::SO3_SPLINE);
        const auto &scaleSpline = _splines->GetRdSpline(Configor::Preference::ACCE_SPLINE);
        // we throw the head and tail data as the rotations from the fitted SO3 Spline in that range are poor
        const double st = std::max(so3Spline.MinTime(), scaleSpline.MinTime()) + Configor::Prior::TimeOffsetPadding;
        const double et = std::min(so3Spline.MaxTime(), scaleSpline.MaxTime()) - Configor::Prior::TimeOffsetPadding;

        constexpr double dt = 0.1;
        std::vector<Eigen::Vector3d> linVelSeq(std::floor((et - st) / dt), Eigen::Vector3d::Zero());
        // we throw the head and tail data as the rotations from the fitted SO3 Spline in that range are poor
        for (int i = 0; i < static_cast<int>(linVelSeq.size()) - 1; ++i) {
            int sIdx = i, eIdx = i + 1;
            double sTime = sIdx * dt + st, eTime = eIdx * dt + st;
            Eigen::Vector3d *sVel = &linVelSeq.at(sIdx), *eVel = &linVelSeq.at(eIdx);

            for (const auto &[topic, _]: Configor::DataStream::IMUTopics) {
                double timeOffset = _parMagr->TEMPORAL.TIME_OFFSET_BiToBr.at(topic);
                // extract data by considering the initialized time offsets
                auto [sIter, eIter] = _dataMagr->ExtractIMUDataPiece(topic, sTime - timeOffset, eTime - timeOffset);

                // vector and matrix sequence for integration
                std::vector<std::pair<double, Eigen::Vector3d>> bVecSeq;
                std::vector<std::pair<double, Eigen::Matrix3d>> AMatSeq;

                auto SO3_BiToBr = _parMagr->EXTRI.SO3_BiToBr.at(topic);

                for (auto iter = sIter; iter != eIter; ++iter) {
                    const auto &frame = *iter;
                    double curTime = frame->GetTimestamp() + timeOffset;

                    if (!_splines->TimeInRangeForSo3(curTime, Configor::Preference::SO3_SPLINE)) { continue; }

                    auto SO3_BrToBr0 = so3Spline.Evaluate(curTime);

                    // angular velocity in world
                    auto SO3_VEL_BrToBr0InBr0 = SO3_BrToBr0 * so3Spline.VelocityBody(curTime);
                    Eigen::Matrix3d SO3_VEL_MAT = Sophus::SO3d::hat(SO3_VEL_BrToBr0InBr0);

                    // angular acceleration in world
                    auto SO3_ACCE_BrToBr0InBr0 = SO3_BrToBr0 * so3Spline.AccelerationBody(curTime);
                    Eigen::Matrix3d SO3_ACCE_MAT = Sophus::SO3d::hat(SO3_ACCE_BrToBr0InBr0);

                    // store
                    bVecSeq.emplace_back(curTime, SO3_BrToBr0 * SO3_BiToBr * frame->GetAcce());
                    AMatSeq.emplace_back(curTime, (SO3_ACCE_MAT + SO3_VEL_MAT * SO3_VEL_MAT) * SO3_BrToBr0.matrix());
                }

                // integration
                Eigen::Vector3d bVec = TrapIntegrationOnce(bVecSeq);
                Eigen::Matrix3d AMat = TrapIntegrationOnce(AMatSeq);

                estimator->AddVelIntegration(
                        topic, bVec, AMat, sVel, eVel, eTime - sTime, optOption, 1.0
                );
            }
        }
        estimator->SetParameterBlockConstant(_parMagr->EXTRI.POS_BiInBr.at(Configor::DataStream::ReferIMU).data());
        sum = estimator->Solve(_ceresOption);

        spdlog::info("here is the summary:\n{}\n", sum.BriefReport());

        // ----------------------------------------------
        // step 3: recover the linear acceleration spline
        // ----------------------------------------------
        spdlog::info("performing linear acceleration spline recovery...");
        estimator = Estimator::Create(_splines, _parMagr);
        optOption = OptOption::Option::OPT_LIN_ACCE_SPLINE;
        // add acceleration residuals
        for (const auto &item: _dataMagr->GetIMUMeasurements(Configor::DataStream::ReferIMU)) {
            estimator->AddIMUAcceMeasurement(
                    item, Configor::DataStream::ReferIMU, optOption, Configor::Prior::Weight::AcceWeight
            );
        }

        sum = estimator->Solve(_ceresOption);

        spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
    }

    void CalibSolver::BatchOptimization(OptOption::Option optOption) {
        auto GetOptString = [](OptOption::Option opt) -> std::string {
            std::stringstream stringStream;
            stringStream << opt;
            return stringStream.str();
        };
        spdlog::info("Optimization option: {}", GetOptString(optOption));

        auto estimator = Estimator::Create(_splines, _parMagr);
        for (const auto &[topic, mes]: _dataMagr->GetIMUMeasurements()) {
            for (const auto &item: mes) {
                // gyro factors
                estimator->AddIMUGyroMeasurement(item, topic, optOption, Configor::Prior::Weight::GyroWeight);
                // acce factors
                estimator->AddIMUAcceMeasurement(item, topic, optOption, Configor::Prior::Weight::AcceWeight);
            }
        }
        estimator->FixFirSO3ControlPoint();
        estimator->SetParameterBlockConstant(_parMagr->EXTRI.SO3_BiToBr.at(Configor::DataStream::ReferIMU).data());
        estimator->SetParameterBlockConstant(_parMagr->EXTRI.POS_BiInBr.at(Configor::DataStream::ReferIMU).data());
        estimator->SetParameterBlockConstant(&_parMagr->TEMPORAL.TIME_OFFSET_BiToBr.at(Configor::DataStream::ReferIMU));

        auto sum = estimator->Solve(_ceresOption);
        spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
    }

    void CalibSolver::SaveBSplines(int hz) const {
        std::string saveDir = Configor::DataStream::OutputPath + "/splines";
        std::filesystem::remove_all(saveDir);
        if (!std::filesystem::create_directories(saveDir)) {
            throw ns_mi::Status(
                    ns_mi::Status::Flag::ERROR,
                    fmt::format("create directory to save trajectories failed: '{}'", saveDir)
            );
        }
        {
            // sampled acceleration points
            const auto &so3Spline = _splines->GetSo3Spline(Configor::Preference::SO3_SPLINE);
            const auto &acceSpline = _splines->GetRdSpline(Configor::Preference::ACCE_SPLINE);
            double st = so3Spline.MinTime(), et = so3Spline.MaxTime(), dt = 1.0 / hz;

            // pose container
            Eigen::aligned_vector<ns_ctraj::Posed> poseSeq;
            poseSeq.reserve(static_cast<std::size_t>((et - st) / dt));

            for (double t = st; t < et;) {
                if (_splines->TimeInRange(t, so3Spline) && _splines->TimeInRange(t, acceSpline)) {
                    Sophus::SO3d curSO3_BrToBr0 = so3Spline.Evaluate(t);
                    Eigen::Vector3d linAcce_BrToBr0InBr0 = acceSpline.Evaluate(t);
                    poseSeq.emplace_back(curSO3_BrToBr0, linAcce_BrToBr0InBr0, t);
                }
                t += dt;
            }
            if (!SavePoseSequence(poseSeq, saveDir + "/b-splines-sample" + ns_mi::Configor::GetFormatExtension(),
                                  ns_mi::Configor::Preference::OutputDataFormat)) {
                throw ns_mi::Status(
                        ns_mi::Status::Flag::ERROR, fmt::format("error occurs when saving b-splines to '{}'", saveDir)
                );
            }
        }
        {
            // control points
            std::ofstream file(saveDir + "/b-splines-cp" + ns_mi::Configor::GetFormatExtension());
            auto ar = GetOutputArchiveVariant(file, ns_mi::Configor::Preference::OutputDataFormat);
            SerializeByOutputArchiveVariant(
                    ar, ns_mi::Configor::Preference::OutputDataFormat,
                    cereal::make_nvp("trajectory", *_splines)
            );
        }
    }

    void CalibSolver::SaveIMUMesEst() const {
        auto &acceSpline = _splines->GetRdSpline(Configor::Preference::ACCE_SPLINE);
        auto &so3Spline = _splines->GetSo3Spline(Configor::Preference::SO3_SPLINE);

        // folder
        std::string saveDir = Configor::DataStream::OutputPath + "/kinematics";
        std::filesystem::remove_all(saveDir);
        if (!std::filesystem::create_directories(saveDir)) {
            throw ns_mi::Status(
                    ns_mi::Status::Flag::ERROR,
                    fmt::format("create directory to save kinematics failed: '{}'", saveDir)
            );
        }

        // align B-spline-derived measurements to each IMU
        for (const auto &[topic, _]: Configor::DataStream::IMUTopics) {

            const IMUIntrinsics &intri = _parMagr->INTRI.IMU.at(topic);
            const auto &SO3_BiToBr = _parMagr->EXTRI.SO3_BiToBr.at(topic);
            const Eigen::Vector3d &POS_BiInBr = _parMagr->EXTRI.POS_BiInBr.at(topic);
            const double &timeOffset = _parMagr->TEMPORAL.TIME_OFFSET_BiToBr.at(topic);

            // inertial measurements
            std::list<ns_mi::IMUFrame> rawMes, estMes, diff;

            for (const auto &item: _dataMagr->GetIMUMeasurements(topic)) {
                double t = item->GetTimestamp() + timeOffset;

                if (!_splines->TimeInRange(t, acceSpline)) { continue; }
                if (!_splines->TimeInRange(t, so3Spline)) { continue; }

                rawMes.push_back(*item);

                const auto &SO3_curBrToW = so3Spline.Evaluate(t);
                const Eigen::Vector3d &angVelInW = SO3_curBrToW * so3Spline.VelocityBody(t);
                const Eigen::Vector3d &angAcceInW = SO3_curBrToW * so3Spline.AccelerationBody(t);
                const Eigen::Matrix3d &angVelMat = Sophus::SO3d::hat(angVelInW);
                const Eigen::Matrix3d &angAcceMat = Sophus::SO3d::hat(angAcceInW);

                const auto &est = IMUIntrinsics::KinematicsToInertialMes(
                        item->GetTimestamp(), acceSpline.Evaluate(t) +
                                              (angAcceMat + angVelMat * angVelMat) * SO3_curBrToW.matrix() * POS_BiInBr,
                        SO3_curBrToW * so3Spline.VelocityBody(t), SO3_curBrToW * SO3_BiToBr, _parMagr->GRAVITY
                );

                estMes.push_back(*intri.InvolveIntri(est));

                diff.emplace_back(
                        item->GetTimestamp(), rawMes.back().GetGyro() - estMes.back().GetGyro(),
                        rawMes.back().GetAcce() - estMes.back().GetAcce()
                );
            }
            std::string name = topic;
            std::replace(name.begin(), name.end(), '/', '_');
            if (name.front() == '_') { name.front() = '/'; }
            std::ofstream file(saveDir + "/" + name + ns_mi::Configor::GetFormatExtension(), std::ios::out);
            auto ar = GetOutputArchiveVariant(file, ns_mi::Configor::Preference::OutputDataFormat);
            SerializeByOutputArchiveVariant(
                    ar, ns_mi::Configor::Preference::OutputDataFormat,
                    cereal::make_nvp("raw_inertial", rawMes), cereal::make_nvp("est_inertial", estMes),
                    cereal::make_nvp("inertial_diff", diff)
            );
        }

        const IMUIntrinsics &refIntri = _parMagr->INTRI.IMU.at(Configor::DataStream::ReferIMU);
        // align inertial measurements of all IMUs to the reference IMU
        for (const auto &[topic, _]: Configor::DataStream::IMUTopics) {

            const IMUIntrinsics &intri = _parMagr->INTRI.IMU.at(topic);
            const auto &SO3_BiToBr = _parMagr->EXTRI.SO3_BiToBr.at(topic);
            const Eigen::Vector3d &POS_BiInBr = _parMagr->EXTRI.POS_BiInBr.at(topic);
            const double &timeOffset = _parMagr->TEMPORAL.TIME_OFFSET_BiToBr.at(topic);

            std::list<ns_mi::IMUFrame> estMes;

            for (const auto &item: _dataMagr->GetIMUMeasurements(topic)) {
                double t = item->GetTimestamp() + timeOffset;

                if (!_splines->TimeInRange(t, acceSpline)) { continue; }
                if (!_splines->TimeInRange(t, so3Spline)) { continue; }

                auto mesInIdeal = intri.RemoveIntri(item);

                const auto &SO3_curBrToW = so3Spline.Evaluate(t);
                const Eigen::Vector3d &angVelInW = SO3_curBrToW * so3Spline.VelocityBody(t);
                const Eigen::Vector3d &angAcceInW = SO3_curBrToW * so3Spline.AccelerationBody(t);
                const Eigen::Matrix3d &angVelMat = Sophus::SO3d::hat(angVelInW);
                const Eigen::Matrix3d &angAcceMat = Sophus::SO3d::hat(angAcceInW);

                auto res = IMUIntrinsics::InertialMesToKinematics(
                        t, mesInIdeal, SO3_curBrToW * SO3_BiToBr, _parMagr->GRAVITY
                );

                auto alignedMes = IMUIntrinsics::KinematicsToInertialMes(
                        std::get<0>(res),
                        std::get<1>(res) - (angAcceMat + angVelMat * angVelMat) * SO3_curBrToW.matrix() * POS_BiInBr,
                        std::get<2>(res),
                        SO3_curBrToW,
                        _parMagr->GRAVITY
                );

                estMes.push_back(*refIntri.InvolveIntri(alignedMes));
            }

            std::string name = topic;
            std::replace(name.begin(), name.end(), '/', '_');
            if (name.front() == '_') { name.front() = '/'; }
            std::ofstream file(saveDir + "/" + name + "_in_ref" + ns_mi::Configor::GetFormatExtension(), std::ios::out);
            auto ar = GetOutputArchiveVariant(file, ns_mi::Configor::Preference::OutputDataFormat);
            SerializeByOutputArchiveVariant(
                    ar, ns_mi::Configor::Preference::OutputDataFormat, cereal::make_nvp("aligned_inertial", estMes)
            );
        }
    }

    CalibSolver::~CalibSolver() {
        // solving is not performed or not finished as an exception is thrown
        if (!_solveFinished) { pangolin::QuitAll(); }
        // solving is finished (when use 'pangolin::QuitAll()', the window not quit immediately)
        while (_viewer->IsActive()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
    }

    // ---------------------
    // MICeresDebugCallBack
    // ---------------------

    MICeresDebugCallBack::MICeresDebugCallBack(CalibParamManager::Ptr calibParamManager)
            : _calibParamManager(std::move(calibParamManager)) {}

    auto MICeresDebugCallBack::Create(const CalibParamManager::Ptr &calibParamManager) {
        return new MICeresDebugCallBack(calibParamManager);
    }

    ceres::CallbackReturnType MICeresDebugCallBack::operator()(const ceres::IterationSummary &summary) {
        // for drawing
        const static std::string paramDir = Configor::DataStream::OutputPath + "/params_iter";
        const std::string iterInfoFilename = paramDir + "/iter_info.csv";

        static int count = 0;
        if (count == 0) {
            std::filesystem::remove_all(paramDir);
            std::filesystem::create_directory(paramDir);

            std::ofstream file(iterInfoFilename, std::ios::out);
            file << "cost,gradient,tr_radius(1/lambda)" << std::endl;
            file.close();
        }
        if (!std::filesystem::exists(paramDir)) {
            spdlog::warn("the directory to save param files (i.e., {}) is invalid!", paramDir);
        } else {
            // save param
            const std::string paramFilename =
                    paramDir + "/params_" + std::to_string(count) + ns_mi::Configor::GetFormatExtension();
            _calibParamManager->Save(paramFilename, ns_mi::Configor::Preference::OutputDataFormat);

            // save iter info
            std::ofstream file(iterInfoFilename, std::ios::app);
            file << count << ',' << summary.cost << ','
                 << summary.gradient_norm << ',' << summary.trust_region_radius << std::endl;
            file.close();

            ++count;
        }
        return ceres::SOLVER_CONTINUE;
    }

    // ----------------------
    // MICeresViewerCallBack
    // ----------------------
    MICeresViewerCallBack::MICeresViewerCallBack(CalibParamManager::Ptr calibParamManager,
                                                 ns_viewer::MultiViewer::Ptr viewer, SplineBundleType::Ptr splines)
            : _parMagr(std::move(calibParamManager)), _viewer(std::move(viewer)), _splines(std::move(splines)) {}

    ceres::CallbackReturnType MICeresViewerCallBack::operator()(const ceres::IterationSummary &summary) {
        _viewer->RemoveEntity(_idVec, CalibSolver::VIEW_SENSORS);
        _viewer->RemoveEntity(_idVec, CalibSolver::VIEW_SPLINE);

        _idVec = AddEntitiesToSensorViewer();
        auto ids = AddEntitiesToSplineViewer();
        _idVec.insert(_idVec.cbegin(), ids.cbegin(), ids.cend());

        return ceres::CallbackReturnType::SOLVER_CONTINUE;
    }

    auto MICeresViewerCallBack::Create(const CalibParamManager::Ptr &calibParamManager,
                                       const ns_viewer::MultiViewer::Ptr &viewer,
                                       const SplineBundleType::Ptr &splines) {
        return std::make_shared<MICeresViewerCallBack>(calibParamManager, viewer, splines);
    }

    std::vector<std::size_t> MICeresViewerCallBack::AddEntitiesToSplineViewer() {
        // spline  viewer
        std::vector<ns_viewer::Entity::Ptr> entities;
        const auto &so3Spline = _splines->GetSo3Spline(Configor::Preference::SO3_SPLINE);
        const auto &acceSpline = _splines->GetRdSpline(Configor::Preference::ACCE_SPLINE);
        double minTime = std::max(so3Spline.MinTime(), acceSpline.MinTime());
        double maxTime = std::min(so3Spline.MaxTime(), acceSpline.MaxTime());
        double dt = 0.005;
        for (double t = minTime; t < maxTime;) {
            if (!_splines->TimeInRange(t, so3Spline) || !_splines->TimeInRange(t, acceSpline)) {
                t += dt;
                continue;
            }

            Sophus::SO3d so3 = so3Spline.Evaluate(t);
            Eigen::Vector3d linScale = acceSpline.Evaluate(t) * Configor::Preference::ScaleSpline;
            // coordinate
            entities.push_back(ns_viewer::Coordinate::Create(
                    ns_viewer::Posed(so3.matrix(), linScale).cast<float>(),
                    static_cast<float>(Configor::Preference::ScaleCoord)
            ));
            t += dt;
        }
        return _viewer->AddEntity(entities, CalibSolver::VIEW_SPLINE);
    }

    std::vector<std::size_t> MICeresViewerCallBack::AddEntitiesToSensorViewer() {
        // sensor suites viewer
        std::vector<ns_viewer::Entity::Ptr> entities;
        auto SE3_BcToBc = Sophus::SE3f();
        auto refIMU = ns_viewer::IMU::Create(
                ns_viewer::Posef(SE3_BcToBc.so3().matrix(), SE3_BcToBc.translation()), 0.1,
                ns_viewer::Colour(0.3f, 0.3f, 0.3f, 1.0f)
        );
        entities.push_back(refIMU);

        for (const auto &[topic, _]: _parMagr->EXTRI.SO3_BiToBr) {
            if (topic == Configor::DataStream::ReferIMU) { continue; }

            auto BiToBc = _parMagr->EXTRI.SE3_BiToBr(topic).cast<float>();
            auto imu = ns_viewer::IMU::Create(
                    ns_viewer::Posef(BiToBc.so3().matrix(), BiToBc.translation()), 0.1
            );
            auto line = ns_viewer::Line::Create(
                    Eigen::Vector3f::Zero(), BiToBc.translation().cast<float>(), ns_viewer::Colour::Black()
            );
            entities.insert(entities.cend(), {imu, line});
        }
        return _viewer->AddEntity(entities, CalibSolver::VIEW_SENSORS);
    }
}