// Copyright (c) 2023. Created on 10/21/23 1:37 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include <utility>
#include "calib/estimator.h"
#include "factor/imu_gyro_factor.hpp"
#include "factor/vel_integration_factor.hpp"
#include "factor/imu_acce_factor.hpp"
#include "factor/vel_factor.hpp"

namespace ns_mi {

    std::shared_ptr<ceres::EigenQuaternionManifold> Estimator::QUATER_MANIFOLD(new ceres::EigenQuaternionManifold());
    std::shared_ptr<ceres::SphereManifold<3>> Estimator::GRAVITY_MANIFOLD(new ceres::SphereManifold<3>());

    ceres::Problem::Options Estimator::DefaultProblemOptions() {
        return ns_ctraj::TrajectoryEstimator<Configor::Prior::SplineOrder>::DefaultProblemOptions();
    }

    ceres::Solver::Options Estimator::DefaultSolverOptions(int threadNum, bool toStdout, bool useCUDA) {
        auto defaultSolverOptions = ns_ctraj::TrajectoryEstimator<Configor::Prior::SplineOrder>::DefaultSolverOptions(
                threadNum, toStdout, useCUDA
        );
        if (!useCUDA) {
            defaultSolverOptions.linear_solver_type = ceres::DENSE_SCHUR;
        }
        defaultSolverOptions.trust_region_strategy_type = ceres::DOGLEG;
        return defaultSolverOptions;
    }

    Estimator::Estimator(SplineBundleType::Ptr splines, CalibParamManager::Ptr calibParamManager)
            : ceres::Problem(DefaultProblemOptions()), splines(std::move(splines)),
              parMagr(std::move(calibParamManager)) {}

    Estimator::Ptr
    Estimator::Create(const SplineBundleType::Ptr &splines, const CalibParamManager::Ptr &calibParamManager) {
        return std::make_shared<Estimator>(splines, calibParamManager);
    }

    ceres::Solver::Summary Estimator::Solve(const ceres::Solver::Options &options) {
        ceres::Solver::Summary summary;
        ceres::Solve(options, this, &summary);
        return summary;
    }

    void Estimator::AddRdKnotsData(std::vector<double *> &paramBlockVec,
                                   const Estimator::SplineBundleType::RdSplineType &spline,
                                   const Estimator::SplineMetaType &splineMeta, bool setToConst) {
        // for each segment
        for (const auto &seg: splineMeta.segments) {
            // the factor 'seg.dt * 0.5' is the treatment for numerical accuracy
            auto idxMaster = spline.ComputeTIndex(seg.t0 + seg.dt * 0.5).second;

            // from the first control point to the last control point
            for (std::size_t i = idxMaster; i < idxMaster + seg.NumParameters(); ++i) {
                auto *data = const_cast<double *>(spline.GetKnot(static_cast<int>(i)).data());

                this->AddParameterBlock(data, 3);

                paramBlockVec.push_back(data);
                // set this param block to be constant
                if (setToConst) { this->SetParameterBlockConstant(data); }
            }
        }
    }

    void Estimator::AddSo3KnotsData(std::vector<double *> &paramBlockVec,
                                    const Estimator::SplineBundleType::So3SplineType &spline,
                                    const Estimator::SplineMetaType &splineMeta, bool setToConst) {
        // for each segment
        for (const auto &seg: splineMeta.segments) {
            // the factor 'seg.dt * 0.5' is the treatment for numerical accuracy
            auto idxMaster = spline.ComputeTIndex(seg.t0 + seg.dt * 0.5).second;

            // from the first control point to the last control point
            for (std::size_t i = idxMaster; i < idxMaster + seg.NumParameters(); ++i) {
                auto *data = const_cast<double *>(spline.GetKnot(static_cast<int>(i)).data());
                // the local parameterization is very important!!!
                this->AddParameterBlock(data, 4, QUATER_MANIFOLD.get());

                paramBlockVec.push_back(data);
                // set this param block to be constant
                if (setToConst) { this->SetParameterBlockConstant(data); }
            }
        }
    }

    Eigen::MatrixXd Estimator::CRSMatrix2EigenMatrix(ceres::CRSMatrix *jacobian_crs_matrix) {
        Eigen::MatrixXd J(jacobian_crs_matrix->num_rows, jacobian_crs_matrix->num_cols);
        J.setZero();

        std::vector<int> jacobian_crs_matrix_rows, jacobian_crs_matrix_cols;
        std::vector<double> jacobian_crs_matrix_values;
        jacobian_crs_matrix_rows = jacobian_crs_matrix->rows;
        jacobian_crs_matrix_cols = jacobian_crs_matrix->cols;
        jacobian_crs_matrix_values = jacobian_crs_matrix->values;

        int cur_index_in_cols_and_values = 0;
        // rows is a num_rows + 1 sized array
        int row_size = static_cast<int>(jacobian_crs_matrix_rows.size()) - 1;
        // outer loop traverse rows, inner loop traverse cols and values
        for (int row_index = 0; row_index < row_size; ++row_index) {
            while (cur_index_in_cols_and_values < jacobian_crs_matrix_rows[row_index + 1]) {
                J(row_index, jacobian_crs_matrix_cols[cur_index_in_cols_and_values]) =
                        jacobian_crs_matrix_values[cur_index_in_cols_and_values];
                cur_index_in_cols_and_values++;
            }
        }
        return J;
    }

    Eigen::MatrixXd Estimator::GetHessianMatrix() {
        ceres::Problem::EvaluateOptions EvalOpts;
        ceres::CRSMatrix jacobian_crs_matrix;
        this->Evaluate(EvalOpts, nullptr, nullptr, nullptr, &jacobian_crs_matrix);
        Eigen::MatrixXd J = CRSMatrix2EigenMatrix(&jacobian_crs_matrix);
        Eigen::MatrixXd H = J.transpose() * J;
        return H;
    }

    /**
     * param blocks:
     * [ SO3 | ... | SO3 | GYRO_BIAS | GYRO_MAP_COEFF | SO3_AtoG | SO3_BiToBr | TIME_OFFSET_BiToBr ]
     */
    void Estimator::AddIMUGyroMeasurement(const IMUFrame::Ptr &imuFrame, const std::string &topic,
                                          Opt option, double gyroWeight) {
        // prepare metas for splines
        SplineMetaType so3Meta;

        // different relative control points finding [single vs. range]
        if (OptOption::IsOptionWith(Opt::OPT_TIME_OFFSET_BiToBr, option)) {
            double minTime = imuFrame->GetTimestamp() - Configor::Prior::TimeOffsetPadding;
            double maxTime = imuFrame->GetTimestamp() + Configor::Prior::TimeOffsetPadding;
            // invalid time stamp
            if (!splines->TimeInRangeForSo3(minTime, Configor::Preference::SO3_SPLINE) ||
                !splines->TimeInRangeForSo3(maxTime, Configor::Preference::SO3_SPLINE)) {
                return;
            }
            splines->CalculateSo3SplineMeta(Configor::Preference::SO3_SPLINE, {{minTime, maxTime}}, so3Meta);
        } else {
            double curTime = imuFrame->GetTimestamp() + parMagr->TEMPORAL.TIME_OFFSET_BiToBr.at(topic);

            // check point time stamp
            if (!splines->TimeInRangeForSo3(curTime, Configor::Preference::SO3_SPLINE)) {
                return;
            }
            splines->CalculateSo3SplineMeta(Configor::Preference::SO3_SPLINE, {{curTime, curTime}}, so3Meta);
        }

        // create a cost function
        auto costFunc = IMUGyroFactor<Configor::Prior::SplineOrder>::Create(so3Meta, imuFrame, gyroWeight);

        // so3 knots param block [each has four sub params]
        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }

        // GYRO gyroBias
        costFunc->AddParameterBlock(3);
        // GYRO map coeff
        costFunc->AddParameterBlock(6);
        // SO3_AtoG
        costFunc->AddParameterBlock(4);
        // SO3_BiToBr
        costFunc->AddParameterBlock(4);
        // TIME_OFFSET_BiToBc
        costFunc->AddParameterBlock(1);

        // set Residuals
        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;

        // so3 knots param block
        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3_SPLINE), so3Meta,
                !OptOption::IsOptionWith(Opt::OPT_SO3_SPLINE, option)
        );

        // GYRO gyroBias
        auto gyroBias = parMagr->INTRI.IMU.at(topic).GYRO.BIAS.data();
        paramBlockVec.push_back(gyroBias);
        // GYRO map coeff
        auto gyroMapCoeff = parMagr->INTRI.IMU.at(topic).GYRO.MAP_COEFF.data();
        paramBlockVec.push_back(gyroMapCoeff);
        // SO3_AtoG
        auto SO3_AtoG = parMagr->INTRI.IMU.at(topic).SO3_AtoG.data();
        paramBlockVec.push_back(SO3_AtoG);
        // SO3_BiToBr
        auto SO3_BiToBr = parMagr->EXTRI.SO3_BiToBr.at(topic).data();
        paramBlockVec.push_back(SO3_BiToBr);
        // TIME_OFFSET_BiToBc
        auto TIME_OFFSET_BiToBc = &parMagr->TEMPORAL.TIME_OFFSET_BiToBr.at(topic);
        paramBlockVec.push_back(TIME_OFFSET_BiToBc);

        // pass to problem
        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);

        this->SetManifold(SO3_AtoG, QUATER_MANIFOLD.get());
        this->SetManifold(SO3_BiToBr, QUATER_MANIFOLD.get());

        if (!OptOption::IsOptionWith(Opt::OPT_GYRO_BIAS, option)) {
            this->SetParameterBlockConstant(gyroBias);
        }

        if (!OptOption::IsOptionWith(Opt::OPT_GYRO_MAP_COEFF, option)) {
            this->SetParameterBlockConstant(gyroMapCoeff);
        }

        if (!OptOption::IsOptionWith(Opt::OPT_SO3_AtoG, option)) {
            this->SetParameterBlockConstant(SO3_AtoG);
        }

        if (!OptOption::IsOptionWith(Opt::OPT_SO3_BiToBr, option)) {
            this->SetParameterBlockConstant(SO3_BiToBr);
        }

        if (!OptOption::IsOptionWith(Opt::OPT_TIME_OFFSET_BiToBr, option)) {
            this->SetParameterBlockConstant(TIME_OFFSET_BiToBc);
        } else {
            // set bound
            this->SetParameterLowerBound(TIME_OFFSET_BiToBc, 0, -Configor::Prior::TimeOffsetPadding);
            this->SetParameterUpperBound(TIME_OFFSET_BiToBc, 0, Configor::Prior::TimeOffsetPadding);
        }
    }

    /**
     * param blocks:
     * [ POS_BiInBr | GRAVITY | START_VEL | END_VEL ]
     */
    void Estimator::AddVelIntegration(const std::string &topic, const Eigen::Vector3d &bVec,
                                      const Eigen::Matrix3d &AMat, Eigen::Vector3d *sVel, Eigen::Vector3d *eVel,
                                      double dt, Estimator::Opt option, double weight) {
        auto costFunc = VelIntegrationFactor::Create(bVec, AMat, dt, weight);
        // POS_BiInBr
        costFunc->AddParameterBlock(3);
        // GRAVITY
        costFunc->AddParameterBlock(3);
        // START_VEL
        costFunc->AddParameterBlock(3);
        // END_VEL
        costFunc->AddParameterBlock(3);

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;

        auto POS_BiInBr = parMagr->EXTRI.POS_BiInBr.at(topic).data();
        paramBlockVec.push_back(POS_BiInBr);

        auto GRAVITY = parMagr->GRAVITY.data();
        paramBlockVec.push_back(GRAVITY);

        paramBlockVec.push_back(sVel->data());
        paramBlockVec.push_back(eVel->data());

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
        this->SetManifold(GRAVITY, GRAVITY_MANIFOLD.get());

        if (!OptOption::IsOptionWith(Opt::OPT_POS_BiInBr, option)) {
            this->SetParameterBlockConstant(POS_BiInBr);
        }
        if (!OptOption::IsOptionWith(Opt::OPT_GRAVITY, option)) {
            this->SetParameterBlockConstant(GRAVITY);
        }
    }

    /**
     * param blocks:
     * [ SO3 | ... | SO3 | LIN_ACCE | ... | LIN_ACCE | ACCE_BIAS | ACCE_MAP_COEFF | GRAVITY |
     *   SO3_BiToBr | POS_BiInBr | TIME_OFFSET_BiToBr ]
     */
    void Estimator::AddIMUAcceMeasurement(const IMUFrame::Ptr &imuFrame, const std::string &topic,
                                          Estimator::Opt option, double acceWeight) {
        // prepare metas for splines
        SplineMetaType so3Meta, acceMeta;

        // different relative control points finding [single vs. range]
        if (OptOption::IsOptionWith(Opt::OPT_TIME_OFFSET_BiToBr, option)) {
            double minTime = imuFrame->GetTimestamp() - Configor::Prior::TimeOffsetPadding;
            double maxTime = imuFrame->GetTimestamp() + Configor::Prior::TimeOffsetPadding;
            // invalid time stamp
            if (!splines->TimeInRangeForSo3(minTime, Configor::Preference::SO3_SPLINE) ||
                !splines->TimeInRangeForSo3(maxTime, Configor::Preference::SO3_SPLINE) ||
                !splines->TimeInRangeForRd(minTime, Configor::Preference::ACCE_SPLINE) ||
                !splines->TimeInRangeForRd(maxTime, Configor::Preference::ACCE_SPLINE)) {
                return;
            }
            splines->CalculateSo3SplineMeta(Configor::Preference::SO3_SPLINE, {{minTime, maxTime}}, so3Meta);
            splines->CalculateRdSplineMeta(Configor::Preference::ACCE_SPLINE, {{minTime, maxTime}}, acceMeta);
        } else {
            double curTime = imuFrame->GetTimestamp() + parMagr->TEMPORAL.TIME_OFFSET_BiToBr.at(topic);

            // check point time stamp
            if (!splines->TimeInRangeForSo3(curTime, Configor::Preference::SO3_SPLINE) ||
                !splines->TimeInRangeForRd(curTime, Configor::Preference::ACCE_SPLINE)) {
                return;
            }
            splines->CalculateSo3SplineMeta(Configor::Preference::SO3_SPLINE, {{curTime, curTime}}, so3Meta);
            splines->CalculateRdSplineMeta(Configor::Preference::ACCE_SPLINE, {{curTime, curTime}}, acceMeta);
        }
        // create a cost function
        auto costFunc = IMUAcceFactor<Configor::Prior::SplineOrder>::Create(
                so3Meta, acceMeta, imuFrame, acceWeight
        );

        // so3 knots param block [each has four sub params]
        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }
        // pos knots param block [each has three sub params]
        for (int i = 0; i < static_cast<int>(acceMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }

        // ACCE_BIAS
        costFunc->AddParameterBlock(3);
        // ACCE_MAP_COEFF
        costFunc->AddParameterBlock(6);
        // GRAVITY
        costFunc->AddParameterBlock(3);
        // SO3_BiToBr
        costFunc->AddParameterBlock(4);
        // POS_BiInBr
        costFunc->AddParameterBlock(3);
        // TIME_OFFSET_BiToBr
        costFunc->AddParameterBlock(1);

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;

        // so3 knots param block
        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3_SPLINE), so3Meta,
                !OptOption::IsOptionWith(Opt::OPT_SO3_SPLINE, option)
        );

        // lin acce knots
        AddRdKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::ACCE_SPLINE), acceMeta,
                !OptOption::IsOptionWith(Opt::OPT_LIN_ACCE_SPLINE, option)
        );

        // ACCE_BIAS
        auto acceBias = parMagr->INTRI.IMU.at(topic).ACCE.BIAS.data();
        paramBlockVec.push_back(acceBias);
        // ACCE_MAP_COEFF
        auto aceMapCoeff = parMagr->INTRI.IMU.at(topic).ACCE.MAP_COEFF.data();
        paramBlockVec.push_back(aceMapCoeff);
        // GRAVITY
        auto gravity = parMagr->GRAVITY.data();
        paramBlockVec.push_back(gravity);
        // SO3_BiToBc
        auto SO3_BiToBc = parMagr->EXTRI.SO3_BiToBr.at(topic).data();
        paramBlockVec.push_back(SO3_BiToBc);
        // POS_BiInBc
        auto POS_BiInBc = parMagr->EXTRI.POS_BiInBr.at(topic).data();
        paramBlockVec.push_back(POS_BiInBc);
        // TIME_OFFSET_BiToBc
        auto TIME_OFFSET_BiToBc = &parMagr->TEMPORAL.TIME_OFFSET_BiToBr.at(topic);
        paramBlockVec.push_back(TIME_OFFSET_BiToBc);

        // pass to problem
        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
        this->SetManifold(gravity, GRAVITY_MANIFOLD.get());
        this->SetManifold(SO3_BiToBc, QUATER_MANIFOLD.get());

        if (!OptOption::IsOptionWith(Opt::OPT_ACCE_BIAS, option)) {
            this->SetParameterBlockConstant(acceBias);
        }

        if (!OptOption::IsOptionWith(Opt::OPT_ACCE_MAP_COEFF, option)) {
            this->SetParameterBlockConstant(aceMapCoeff);
        }

        if (!OptOption::IsOptionWith(Opt::OPT_GRAVITY, option)) {
            this->SetParameterBlockConstant(gravity);
        }

        if (!OptOption::IsOptionWith(Opt::OPT_SO3_BiToBr, option)) {
            this->SetParameterBlockConstant(SO3_BiToBc);
        }

        if (!OptOption::IsOptionWith(Opt::OPT_POS_BiInBr, option)) {
            this->SetParameterBlockConstant(POS_BiInBc);
        }

        if (!OptOption::IsOptionWith(Opt::OPT_TIME_OFFSET_BiToBr, option)) {
            this->SetParameterBlockConstant(TIME_OFFSET_BiToBc);
        } else {
            // set bound
            this->SetParameterLowerBound(TIME_OFFSET_BiToBc, 0, -Configor::Prior::TimeOffsetPadding);
            this->SetParameterUpperBound(TIME_OFFSET_BiToBc, 0, Configor::Prior::TimeOffsetPadding);
        }
    }

    void Estimator::FixFirSO3ControlPoint() {
        const auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3_SPLINE);
        for (int i = 0; i < static_cast<int>(so3Spline.GetKnots().size()); ++i) {
            auto data = so3Spline.GetKnot(i).data();
            if (this->HasParameterBlock(data)) {
                this->SetParameterBlockConstant(data);
                break;
            }
        }
    }

    void Estimator::FixSO3ControlPointAt(int idx) {
        const auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3_SPLINE);
        auto data = so3Spline.GetKnot(idx).data();
        if (this->HasParameterBlock(data)) {
            this->SetParameterBlockConstant(data);
        }
    }

    /**
     * param blocks:
     * [ VEL | ... | VEL ]
     */
    void Estimator::AddVelConstraint(double time, const Eigen::Vector3d &velocity,
                                     Estimator::Opt option, double weight) {
        // prepare metas for splines
        SplineMetaType acceMeta;
        splines->CalculateRdSplineMeta(Configor::Preference::ACCE_SPLINE, {{time, time}}, acceMeta);

        // create a cost function
        auto costFunc = LinVelFactor<Configor::Prior::SplineOrder>::Create(
                acceMeta, time, velocity, weight
        );

        // pos knots param block [each has three sub params]
        for (int i = 0; i < static_cast<int>(acceMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;

        // lin acce knots (Pretend this is a velocity spline)
        AddRdKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::ACCE_SPLINE), acceMeta,
                !OptOption::IsOptionWith(Opt::OPT_LIN_ACCE_SPLINE, option)
        );

        // pass to problem
        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
    }
}