// Copyright (c) 2023. Created on 7/7/23 1:29 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef MI_CALIB_CALIB_SOLVER_H
#define MI_CALIB_CALIB_SOLVER_H

#include <utility>
#include "calib/calib_param_manager.h"
#include "calib/calib_data_manager.h"
#include "ctraj/core/trajectory.h"
#include "ceres/ceres.h"
#include "tiny-viewer/core/multi_viewer.h"
#include "ctraj/core/spline_bundle.h"
#include "calib/estimator.h"

namespace ns_mi {

    class CalibSolver {
    public:
        using Ptr = std::shared_ptr<CalibSolver>;
        using SplineBundleType = ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>;

        const static std::string VIEW_SENSORS, VIEW_SPLINE;

    private:
        CalibDataManager::Ptr _dataMagr;

        CalibParamManager::Ptr _parMagr;

        SplineBundleType::Ptr _splines;

        ceres::Solver::Options _ceresOption;

        ns_viewer::MultiViewer::Ptr _viewer;

        bool _solveFinished;

    public:
        explicit CalibSolver(CalibDataManager::Ptr calibDataManager, CalibParamManager::Ptr calibParamManager);

        static CalibSolver::Ptr
        Create(const CalibDataManager::Ptr &calibDataManager, const CalibParamManager::Ptr &calibParamManager);

        void Process();

        void SaveBSplines(int hz = 100) const;

        void SaveIMUMesEst() const;

        virtual ~CalibSolver();

    protected:
        void Initialization();

        void BatchOptimization(OptOption::Option optOption);
    };


    struct MICeresDebugCallBack : public ceres::IterationCallback {
        CalibParamManager::Ptr _calibParamManager;

        explicit MICeresDebugCallBack(CalibParamManager::Ptr calibParamManager);

        static auto Create(const CalibParamManager::Ptr &calibParamManager);

        ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) override;
    };

    struct MICeresViewerCallBack : public ceres::IterationCallback {
        using SplineBundleType = ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>;

        CalibParamManager::Ptr _parMagr;
        ns_viewer::MultiViewer::Ptr _viewer;
        SplineBundleType::Ptr _splines;
        std::vector<std::size_t> _idVec;

        explicit MICeresViewerCallBack(CalibParamManager::Ptr calibParamManager,
                                       ns_viewer::MultiViewer::Ptr viewer,
                                       SplineBundleType::Ptr splines);

        static auto Create(const CalibParamManager::Ptr &calibParamManager,
                           const ns_viewer::MultiViewer::Ptr &viewer,
                           const SplineBundleType::Ptr &splines);

        ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) override;

        std::vector<std::size_t> AddEntitiesToSensorViewer();

        std::vector<std::size_t> AddEntitiesToSplineViewer();
    };

}

#endif //MI_CALIB_CALIB_SOLVER_H
