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
