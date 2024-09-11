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

#ifndef MI_CALIB_VEL_INTEGRATION_FACTOR_HPP
#define MI_CALIB_VEL_INTEGRATION_FACTOR_HPP

#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"

namespace ns_mi {
    struct VelIntegrationFactor {
    private:
        Eigen::Vector3d bVec;
        Eigen::Matrix3d AMat;
        double dt;
        double weight;

    public:
        VelIntegrationFactor(Eigen::Vector3d bVec, Eigen::Matrix3d aMat, double dt, double weight)
                : bVec(std::move(bVec)), AMat(std::move(aMat)), dt(dt), weight(weight) {}

        static auto Create(const Eigen::Vector3d &bVec, const Eigen::Matrix3d &aMat, double dt, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<VelIntegrationFactor>(
                    new VelIntegrationFactor(bVec, aMat, dt, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(VelIntegrationFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ POS_BiInBr | GRAVITY | START_VEL | END_VEL ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            Eigen::Map<const Eigen::Vector3<T>> POS_BiInBr(sKnots[0]);
            Eigen::Map<const Eigen::Vector3<T>> GRAVITY(sKnots[1]);
            Eigen::Map<const Eigen::Vector3<T>> START_VEL(sKnots[2]);
            Eigen::Map<const Eigen::Vector3<T>> END_VEL(sKnots[3]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = (bVec - AMat * POS_BiInBr) - (END_VEL - START_VEL - GRAVITY * dt);
            residuals = T(weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //MI_CALIB_VEL_INTEGRATION_FACTOR_HPP
