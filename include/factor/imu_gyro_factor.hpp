// Copyright (c) 2023. Created on 7/7/23 1:29 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef MI_CALIB_IMU_GYRO_FACTOR_HPP
#define MI_CALIB_IMU_GYRO_FACTOR_HPP

#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace ns_mi {
    template<int Order>
    struct IMUGyroFactor {
    private:
        ns_ctraj::SplineMeta<Order> _splineMeta;
        IMUFrame::Ptr _imuFrame{};

        double _dtInv;
        double _weight;

    public:
        explicit IMUGyroFactor(ns_ctraj::SplineMeta<Order> splineMeta, IMUFrame::Ptr imuFrame, double weight)
                : _splineMeta(std::move(splineMeta)), _imuFrame(std::move(imuFrame)),
                  _dtInv(1.0 / _splineMeta.segments.front().dt), _weight(weight) {}

        static auto
        Create(const ns_ctraj::SplineMeta<Order> &splineMeta, const IMUFrame::Ptr &imuFrame, double gyroWeight) {
            return new ceres::DynamicAutoDiffCostFunction<IMUGyroFactor>(
                    new IMUGyroFactor(splineMeta, imuFrame, gyroWeight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(IMUGyroFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | GYRO_BIAS | GYRO_MAP_COEFF | SO3_AtoG | SO3_BiToBr | TIME_OFFSET_BiToBr ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            // array offset
            std::size_t SO3_OFFSET;
            std::size_t GYRO_BIAS_OFFSET = _splineMeta.NumParameters();
            std::size_t GYRO_MAP_COEFF_OFFSET = GYRO_BIAS_OFFSET + 1;
            std::size_t SO3_AtoG_OFFSET = GYRO_MAP_COEFF_OFFSET + 1;
            std::size_t SO3_BiToBr_OFFSET = SO3_AtoG_OFFSET + 1;
            std::size_t TIME_OFFSET_BiToBr_OFFSET = SO3_BiToBr_OFFSET + 1;

            T TIME_OFFSET_BiToBr = sKnots[TIME_OFFSET_BiToBr_OFFSET][0];


            auto timeByBr = _imuFrame->GetTimestamp() + TIME_OFFSET_BiToBr;

            // calculate the so3 offset
            std::pair<std::size_t, T> pointIU;
            _splineMeta.template ComputeSplineIndex(timeByBr, pointIU.first, pointIU.second);
            SO3_OFFSET = pointIU.first;

            Sophus::SO3<T> SO3_BrToBr0;
            Sophus::SO3Tangent<T> SO3_VEL_BrToBr0InBr;
            ns_ctraj::CeresSplineHelperJet<T, Order>::template EvaluateLie(
                    sKnots + SO3_OFFSET, pointIU.second, _dtInv, &SO3_BrToBr0, &SO3_VEL_BrToBr0InBr
            );

            Eigen::Map<const Eigen::Vector3<T>> gyroBias(sKnots[GYRO_BIAS_OFFSET]);
            auto gyroCoeff = sKnots[GYRO_MAP_COEFF_OFFSET];
            Eigen::Matrix33<T> gyroMapMat = Eigen::Matrix33<T>::Zero();
            gyroMapMat.diagonal() = Eigen::Map<const Eigen::Vector3<T>>(gyroCoeff, 3);
            gyroMapMat(0, 1) = *(gyroCoeff + 3);
            gyroMapMat(0, 2) = *(gyroCoeff + 4);
            gyroMapMat(1, 2) = *(gyroCoeff + 5);

            Eigen::Map<Sophus::SO3<T> const> const SO3_AtoG(sKnots[SO3_AtoG_OFFSET]);
            Eigen::Map<Sophus::SO3<T> const> const SO3_BiToBr(sKnots[SO3_BiToBr_OFFSET]);
            Sophus::SO3<T> SO3_BiToBr0 = SO3_BrToBr0 * SO3_BiToBr;
            Sophus::SO3Tangent<T> SO3_VEL_BrToBr0InBr0 = SO3_BrToBr0 * SO3_VEL_BrToBr0InBr;

            Eigen::Vector3<T> pred =
                    (gyroMapMat * (SO3_AtoG * SO3_BiToBr0.inverse() * SO3_VEL_BrToBr0InBr0)).eval() + gyroBias;

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = pred - _imuFrame->GetGyro().template cast<T>();
            residuals = T(_weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //MI_CALIB_IMU_GYRO_FACTOR_HPP
