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

#ifndef MI_CALIB_VEL_FACTOR_HPP
#define MI_CALIB_VEL_FACTOR_HPP

#include <utility>

#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"

namespace ns_mi {
    template<int Order>
    struct LinVelFactor {
    private:
        ns_ctraj::SplineMeta<Order> _splineMeta;

        double _time;
        Eigen::Vector3d _vel;
        double _dtInv;
        double _weight;

    public:
        LinVelFactor(ns_ctraj::SplineMeta<Order> splineMeta, double time, Eigen::Vector3d velocity, double weight)
                : _splineMeta(std::move(splineMeta)), _time(time), _vel(std::move(velocity)),
                  _dtInv(1.0 / _splineMeta.segments.front().dt), _weight(weight) {}

        static auto Create(const ns_ctraj::SplineMeta<Order> &splineMeta, double time,
                           const Eigen::Vector3d &velocity, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<LinVelFactor>(
                    new LinVelFactor(splineMeta, time, velocity, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(LinVelFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ VEL | ... | VEL ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            // array offset
            std::size_t POS_OFFSET;
            double u;
            _splineMeta.template ComputeSplineIndex(_time, POS_OFFSET, u);

            Eigen::Vector3<T> pred_LIN_VEL_BrToBr0InBr0;

            // Pretend this is a velocity spline
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    sKnots + POS_OFFSET, u, _dtInv, &pred_LIN_VEL_BrToBr0InBr0
            );

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);

            residuals = pred_LIN_VEL_BrToBr0InBr0 - _vel.template cast<T>();

            residuals = T(_weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //MI_CALIB_VEL_FACTOR_HPP
