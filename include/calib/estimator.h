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

#ifndef MI_CALIB_ESTIMATOR_H
#define MI_CALIB_ESTIMATOR_H

#include "ctraj/core/spline_bundle.h"
#include "config/configor.h"
#include "ctraj/core/trajectory_estimator.h"
#include "sensor/imu.h"
#include "calib/calib_param_manager.h"

namespace ns_mi {
    using namespace magic_enum::bitwise_operators;

    struct OptOption {
        // myenumGenor Option OPT_SO3_SPLINE OPT_LIN_ACCE_SPLINE OPT_SO3_BiToBr OPT_POS_BiInBr OPT_TIME_OFFSET_BiToBr OPT_GYRO_BIAS OPT_GYRO_MAP_COEFF OPT_ACCE_BIAS OPT_ACCE_MAP_COEFF OPT_SO3_AtoG OPT_GRAVITY
        enum class Option : std::uint32_t {
            /**
             * @brief options
             */
            NONE = 1 << 0,
            OPT_SO3_SPLINE = 1 << 1,
            OPT_LIN_ACCE_SPLINE = 1 << 2,

            OPT_SO3_BiToBr = 1 << 3,
            OPT_POS_BiInBr = 1 << 4,

            OPT_TIME_OFFSET_BiToBr = 1 << 5,

            OPT_GYRO_BIAS = 1 << 6,
            OPT_GYRO_MAP_COEFF = 1 << 7,

            OPT_ACCE_BIAS = 1 << 8,
            OPT_ACCE_MAP_COEFF = 1 << 9,
            OPT_SO3_AtoG = 1 << 10,

            OPT_GRAVITY = 1 << 11,

            ALL = OPT_SO3_SPLINE | OPT_LIN_ACCE_SPLINE | OPT_SO3_BiToBr | OPT_POS_BiInBr |
                  OPT_TIME_OFFSET_BiToBr | OPT_GYRO_BIAS | OPT_GYRO_MAP_COEFF |
                  OPT_ACCE_BIAS | OPT_ACCE_MAP_COEFF | OPT_SO3_AtoG | OPT_GRAVITY
        };

        static bool IsOptionWith(Option desired, Option curOption) {
            return (desired == (desired & curOption));
        }

        /**
         * @brief override operator '<<' for type 'Option'
         */
        friend std::ostream &operator<<(std::ostream &os, const Option &curOption) {
            std::stringstream stream;
            int count = 0;
            if (IsOptionWith(Option::OPT_SO3_SPLINE, curOption)) {
                stream << "OPT_SO3_SPLINE";
                ++count;
            }
            if (IsOptionWith(Option::OPT_LIN_ACCE_SPLINE, curOption)) {
                stream << " | OPT_LIN_ACCE_SPLINE";
                ++count;
            }
            if (IsOptionWith(Option::OPT_SO3_BiToBr, curOption)) {
                stream << " | OPT_SO3_BiToBr";
                ++count;
            }
            if (IsOptionWith(Option::OPT_POS_BiInBr, curOption)) {
                stream << " | OPT_POS_BiInBr";
                ++count;
            }
            if (IsOptionWith(Option::OPT_TIME_OFFSET_BiToBr, curOption)) {
                stream << " | OPT_TIME_OFFSET_BiToBr";
                ++count;
            }
            if (IsOptionWith(Option::OPT_GYRO_BIAS, curOption)) {
                stream << " | OPT_GYRO_BIAS";
                ++count;
            }
            if (IsOptionWith(Option::OPT_GYRO_MAP_COEFF, curOption)) {
                stream << " | OPT_GYRO_MAP_COEFF";
                ++count;
            }
            if (IsOptionWith(Option::OPT_ACCE_BIAS, curOption)) {
                stream << " | OPT_ACCE_BIAS";
                ++count;
            }
            if (IsOptionWith(Option::OPT_ACCE_MAP_COEFF, curOption)) {
                stream << " | OPT_ACCE_MAP_COEFF";
                ++count;
            }
            if (IsOptionWith(Option::OPT_SO3_AtoG, curOption)) {
                stream << " | OPT_SO3_AtoG";
                ++count;
            }
            if (IsOptionWith(Option::OPT_GRAVITY, curOption)) {
                stream << " | OPT_GRAVITY";
                ++count;
            }
            if (count == 0) {
                os << "NONE";
            } else if (count == 11) {
                os << "ALL";
            } else {
                std::string str = stream.str();
                if (str.at(1) == '|') {
                    str = str.substr(3, str.size() - 3);
                }
                os << str;
            }
            return os;
        };
    };

    class Estimator : public ceres::Problem {
    public:
        using Ptr = std::shared_ptr<Estimator>;
        using SplineBundleType = ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>;
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;
        using Opt = OptOption::Option;

    private:
        SplineBundleType::Ptr splines;
        CalibParamManager::Ptr parMagr;

        // manifolds
        static std::shared_ptr<ceres::EigenQuaternionManifold> QUATER_MANIFOLD;
        static std::shared_ptr<ceres::SphereManifold<3>> GRAVITY_MANIFOLD;

    public:
        Estimator(SplineBundleType::Ptr splines, CalibParamManager::Ptr calibParamManager);

        static Ptr Create(const SplineBundleType::Ptr &splines, const CalibParamManager::Ptr &calibParamManager);

        static ceres::Problem::Options DefaultProblemOptions();

        static ceres::Solver::Options
        DefaultSolverOptions(int threadNum = -1, bool toStdout = true, bool useCUDA = false);

        ceres::Solver::Summary Solve(const ceres::Solver::Options &options = Estimator::DefaultSolverOptions());

        Eigen::MatrixXd GetHessianMatrix();

    public:
        void AddIMUGyroMeasurement(const IMUFrame::Ptr &imuFrame, const std::string &topic,
                                   Opt option, double gyroWeight);

        void AddVelIntegration(const std::string &topic, const Eigen::Vector3d &bVec, const Eigen::Matrix3d &AMat,
                               Eigen::Vector3d *sVel, Eigen::Vector3d *eVel, double dt, Opt option, double weight);

        void AddIMUAcceMeasurement(const IMUFrame::Ptr &imuFrame, const std::string &topic,
                                   Opt option, double acceWeight);

        void AddVelConstraint(double time, const Eigen::Vector3d &velocity, Opt option, double weight);

        void FixFirSO3ControlPoint();

        void FixSO3ControlPointAt(int idx);

    protected:
        void AddSo3KnotsData(std::vector<double *> &paramBlockVec, const SplineBundleType::So3SplineType &spline,
                             const SplineMetaType &splineMeta, bool setToConst);

        void AddRdKnotsData(std::vector<double *> &paramBlockVec, const SplineBundleType::RdSplineType &spline,
                            const SplineMetaType &splineMeta, bool setToConst);

        static Eigen::MatrixXd CRSMatrix2EigenMatrix(ceres::CRSMatrix *jacobian_crs_matrix);
    };
}

#endif //MI_CALIB_ESTIMATOR_H
