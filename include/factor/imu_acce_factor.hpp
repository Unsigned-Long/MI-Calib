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

#ifndef MI_CALIB_IMU_ACCE_FACTOR_HPP
#define MI_CALIB_IMU_ACCE_FACTOR_HPP

#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace ns_mi {
    template<int Order>
    struct IMUAcceFactor {
    private:
        ns_ctraj::SplineMeta<Order> _so3Meta, _linAcceMeta;
        IMUFrame::Ptr _imuFrame{};

        double _rotDtInv, _linAcceDtInv;
        double _weight;
    public:
        explicit IMUAcceFactor(ns_ctraj::SplineMeta<Order> rotMeta, ns_ctraj::SplineMeta<Order> linAcceMeta,
                               IMUFrame::Ptr imuFrame, double weight)
                : _so3Meta(rotMeta), _linAcceMeta(std::move(linAcceMeta)), _imuFrame(std::move(imuFrame)),
                  _rotDtInv(1.0 / rotMeta.segments.front().dt),
                  _linAcceDtInv(1.0 / _linAcceMeta.segments.front().dt), _weight(weight) {}

        static auto
        Create(const ns_ctraj::SplineMeta<Order> &rotMeta, const ns_ctraj::SplineMeta<Order> &splineMeta,
               const IMUFrame::Ptr &imuFrame, double acceWeight) {
            return new ceres::DynamicAutoDiffCostFunction<IMUAcceFactor>(
                    new IMUAcceFactor(rotMeta, splineMeta, imuFrame, acceWeight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(IMUAcceFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | LIN_ACCE | ... | LIN_ACCE | ACCE_BIAS | ACCE_MAP_COEFF | GRAVITY |
         *   SO3_BiToBr | POS_BiInBr | TIME_OFFSET_BiToBr ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            std::size_t ROT_offset;
            std::size_t LIN_ACCE_offset;

            std::size_t ACCE_BIAS_OFFSET = _so3Meta.NumParameters() + _linAcceMeta.NumParameters();
            std::size_t ACCE_MAP_COEFF_OFFSET = ACCE_BIAS_OFFSET + 1;
            std::size_t GRAVITY_OFFSET = ACCE_MAP_COEFF_OFFSET + 1;
            std::size_t SO3_BiToBr_OFFSET = GRAVITY_OFFSET + 1;
            std::size_t POS_BiInBr_OFFSET = SO3_BiToBr_OFFSET + 1;
            std::size_t TIME_OFFSET_BiToBr_OFFSET = POS_BiInBr_OFFSET + 1;

            // get value
            Eigen::Map<const Sophus::SO3<T>> SO3_BiToBr(sKnots[SO3_BiToBr_OFFSET]);
            Eigen::Map<const Eigen::Vector3<T>> POS_BiInBr(sKnots[POS_BiInBr_OFFSET]);
            T TIME_OFFSET_BiToBr = sKnots[TIME_OFFSET_BiToBr_OFFSET][0];

            auto timeByBr = _imuFrame->GetTimestamp() + TIME_OFFSET_BiToBr;

            // calculate the so3 and pos offset
            std::pair<std::size_t, T> rotPointIU, linAccePointIU;
            _so3Meta.template ComputeSplineIndex(timeByBr, rotPointIU.first, rotPointIU.second);
            _linAcceMeta.template ComputeSplineIndex(timeByBr, linAccePointIU.first, linAccePointIU.second);

            ROT_offset = rotPointIU.first;
            LIN_ACCE_offset = linAccePointIU.first + _so3Meta.NumParameters();

            Sophus::SO3<T> SO3_BrToBr0;
            Sophus::SO3Tangent<T> SO3_VEL_BrToBr0InBr, SO3_ACCE_BrToBr0InBr;
            ns_ctraj::CeresSplineHelperJet<T, Order>::template EvaluateLie(
                    sKnots + ROT_offset, rotPointIU.second, _rotDtInv,
                    &SO3_BrToBr0, &SO3_VEL_BrToBr0InBr, &SO3_ACCE_BrToBr0InBr
            );
            Sophus::SO3Tangent<T> SO3_VEL_BrToBr0InBr0 = SO3_BrToBr0 * SO3_VEL_BrToBr0InBr;
            Sophus::SO3Tangent<T> SO3_ACCE_BrToBr0InBr0 = SO3_BrToBr0 * SO3_ACCE_BrToBr0InBr;

            /**
             * @attention: current R^3 trajectory is the velocity b-spline, whose
             * first order derivative is the linear acceleration, not the second order derivative!!!
             */
            Eigen::Vector3<T> ACCE_BrToBr0InBr0;
            ns_ctraj::CeresSplineHelperJet<T, Order>::template Evaluate<3, 0>(
                    sKnots + LIN_ACCE_offset, linAccePointIU.second, _linAcceDtInv, &ACCE_BrToBr0InBr0
            );

            Eigen::Map<const Eigen::Vector3<T>> acceBias(sKnots[ACCE_BIAS_OFFSET]);
            Eigen::Map<const Eigen::Vector3<T>> gravity(sKnots[GRAVITY_OFFSET]);

            auto acceCoeff = sKnots[ACCE_MAP_COEFF_OFFSET];

            Eigen::Matrix33<T> acceMapMat = Eigen::Matrix33<T>::Zero();

            acceMapMat.diagonal() = Eigen::Map<const Eigen::Vector3<T>>(acceCoeff, 3);
            acceMapMat(0, 1) = *(acceCoeff + 3);
            acceMapMat(0, 2) = *(acceCoeff + 4);
            acceMapMat(1, 2) = *(acceCoeff + 5);

            Sophus::SO3<T> SO3_BiToBr0 = SO3_BrToBr0 * SO3_BiToBr;

            Eigen::Matrix33<T> SO3_VEL_MAT = Sophus::SO3<T>::hat(SO3_VEL_BrToBr0InBr0);
            Eigen::Matrix33<T> SO3_ACCE_MAT = Sophus::SO3<T>::hat(SO3_ACCE_BrToBr0InBr0);
            Eigen::Vector3<T> POS_ACCE_BiToBr0InBr0 =
                    ACCE_BrToBr0InBr0 +
                    (SO3_ACCE_MAT + SO3_VEL_MAT * SO3_VEL_MAT) * (SO3_BrToBr0.matrix() * POS_BiInBr);

            // Eigen::Vector3<T> POS_ACCE_BiToBr0InBr0 =
            //         -Sophus::SO3<T>::hat(SO3_BrToBr0 * POS_BiInBr) * SO3_ACCE_BrToBr0InBr0 +
            //         ACCE_BrToBr0InBr0 - Sophus::SO3<T>::hat(SO3_VEL_BrToBr0InBr0) *
            //                             Sophus::SO3<T>::hat(SO3_BrToBr0 * POS_BiInBr) *
            //                             SO3_VEL_BrToBr0InBr0;

            Eigen::Vector3<T> accePred =
                    (acceMapMat * (SO3_BiToBr0.inverse() * (POS_ACCE_BiToBr0InBr0 - gravity))).eval() + acceBias;

            Eigen::Vector3<T> acceResiduals = accePred - _imuFrame->GetAcce().template cast<T>();

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = T(_weight) * acceResiduals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
#endif //MI_CALIB_IMU_ACCE_FACTOR_HPP
