// Copyright (c) 2023. Created on 10/9/23 9:18 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef MI_CALIB_IMU_INTRINSIC_HPP
#define MI_CALIB_IMU_INTRINSIC_HPP

#include "util/utils.hpp"
#include "sensor/imu.h"

namespace ns_mi {

    struct BiasMapCoeff {
        Eigen::Vector3d BIAS;
        /**
         * MAP_COEFF: [v1, v2, v3, v4, v5, v6]^T
         * mapMatrix:
         *   v1 & v4 & v5
         *    0 & v2 & v6
         *    0 &  0 & v3
         * f(measure) = mapMat * f(real) + bias
         */
        Eigen::Vector6d MAP_COEFF;

        // organize the vector to a matrix
        [[nodiscard]] Eigen::Matrix3d MapMatrix() const {
            Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
            mat(0, 0) = MAP_COEFF(0), mat(1, 1) = MAP_COEFF(1), mat(2, 2) = MAP_COEFF(2);
            mat(0, 1) = MAP_COEFF(3);
            mat(0, 2) = MAP_COEFF(4);
            mat(1, 2) = MAP_COEFF(5);
            return mat;
        }

        void Clear() {
            BIAS = Eigen::Vector3d::Zero();
            MAP_COEFF = Eigen::Vector6d::Zero();
            MAP_COEFF(0) = 1.0;
            MAP_COEFF(1) = 1.0;
            MAP_COEFF(2) = 1.0;
        }

        // Serialization
        template<class Archive>
        void serialize(Archive &archive) {
            archive(CEREAL_NVP(BIAS), CEREAL_NVP(MAP_COEFF));
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct IMUIntrinsics {
        using Ptr = std::shared_ptr<IMUIntrinsics>;

        // trans radian angle to degree angle
        constexpr static double RAD_TO_DEG = 180.0 / M_PI;
        // trans degree angle to radian angle
        constexpr static double DEG_TO_RAD = M_PI / 180.0;

        BiasMapCoeff ACCE, GYRO;
        Sophus::SO3d SO3_AtoG;

        IMUIntrinsics() { Clear(); }

        static Ptr Create() {
            return std::make_shared<IMUIntrinsics>();
        }

        void Clear() {
            ACCE.Clear();
            GYRO.Clear();
            SO3_AtoG = Sophus::SO3d();
        }

        [[nodiscard]] static ns_mi::IMUFrame::Ptr
        KinematicsToInertialMes(double time, const Eigen::Vector3d &linAcceInW,
                                const Eigen::Vector3d &angVelInW,
                                const Sophus::SO3d &so3CurToW,
                                const Eigen::Vector3d &gravityInW) {
            Eigen::Vector3d force = so3CurToW.inverse() * (linAcceInW - gravityInW);
            Eigen::Vector3d angVel = so3CurToW.inverse() * angVelInW;
            return IMUFrame::Create(time, angVel, force);
        }

        [[nodiscard]] static std::tuple<double, Eigen::Vector3d, Eigen::Vector3d>
        InertialMesToKinematics(double time, const ns_mi::IMUFrame::Ptr &frame,
                                const Sophus::SO3d &so3CurToW,
                                const Eigen::Vector3d &gravityInW) {
            Eigen::Vector3d acceInW = so3CurToW * frame->GetAcce() + gravityInW;
            Eigen::Vector3d angVelInW = so3CurToW * frame->GetGyro();
            return {time, acceInW, angVelInW};
        }

        [[nodiscard]] Eigen::Vector3d InvolveForceIntri(const Eigen::Vector3d &force) const {
            return ACCE.MapMatrix() * force + ACCE.BIAS;
        }

        [[nodiscard]] Eigen::Vector3d InvolveGyroIntri(const Eigen::Vector3d &gyro) const {
            return GYRO.MapMatrix() * (SO3_AtoG * gyro) + GYRO.BIAS;
        }

        [[nodiscard]] IMUFrame::Ptr InvolveIntri(const IMUFrame::Ptr &frame) const {
            return IMUFrame::Create(
                    frame->GetTimestamp(), InvolveGyroIntri(frame->GetGyro()), InvolveForceIntri(frame->GetAcce())
            );
        }

        [[nodiscard]] Eigen::Vector3d RemoveForceIntri(const Eigen::Vector3d &force) const {
            return ACCE.MapMatrix().inverse() * (force - ACCE.BIAS);
        }

        [[nodiscard]] Eigen::Vector3d RemoveGyroIntri(const Eigen::Vector3d &gyro) const {
            return SO3_AtoG.inverse().matrix() * GYRO.MapMatrix().inverse() * (gyro - GYRO.BIAS);
        }

        [[nodiscard]] IMUFrame::Ptr RemoveIntri(const IMUFrame::Ptr &frame) const {
            return IMUFrame::Create(
                    frame->GetTimestamp(), RemoveGyroIntri(frame->GetGyro()), RemoveForceIntri(frame->GetAcce())
            );
        }

        // quaternion
        [[nodiscard]] Eigen::Quaterniond Q_AtoG() const {
            return SO3_AtoG.unit_quaternion();
        }

        // euler angles
        [[nodiscard]] Eigen::Vector3d EULER_AtoG_RAD() const {
            return Q_AtoG().toRotationMatrix().eulerAngles(0, 1, 2);
        }

        [[nodiscard]] Eigen::Vector3d EULER_AtoG_DEG() const {
            auto euler = EULER_AtoG_RAD();
            for (int i = 0; i != 3; ++i) { euler(i) *= IMUIntrinsics::RAD_TO_DEG; }
            return euler;
        }

        // save the parameters to file using cereal library
        void
        Save(const std::string &filename, CerealArchiveType::Enum archiveType = CerealArchiveType::Enum::YAML) const {
            std::ofstream file(filename, std::ios::out);
            auto ar = GetOutputArchiveVariant(file, archiveType);
            SerializeByOutputArchiveVariant(ar, archiveType, cereal::make_nvp("Intrinsics", *this));
        }

        // load the parameters from file using cereal library
        static IMUIntrinsics::Ptr
        Load(const std::string &filename, CerealArchiveType::Enum archiveType = CerealArchiveType::Enum::YAML) {
            auto intri = IMUIntrinsics::Create();
            std::ifstream file(filename, std::ios::in);
            auto ar = GetInputArchiveVariant(file, archiveType);
            SerializeByInputArchiveVariant(ar, archiveType, cereal::make_nvp("Intrinsics", *intri));
            return intri;
        }


        // Serialization
        template<class Archive>
        void serialize(Archive &archive) {
            archive(CEREAL_NVP(ACCE), CEREAL_NVP(GYRO), CEREAL_NVP(SO3_AtoG));
        }
    };

}
#endif //MI_CALIB_IMU_INTRINSIC_HPP
