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

#ifndef MI_CALIB_VELOCITY_INTEGRATION_HPP
#define MI_CALIB_VELOCITY_INTEGRATION_HPP

#include "ctraj/core/simu_trajectory.h"
#include "sensor/imu.h"
#include "factor/vel_integration_factor.hpp"
#include "calib/estimator.h"

namespace ns_mi {
    struct VelIntegrationLearner {
        static Sophus::SE3d GeneratePoseBias() {
            constexpr double DEG_TO_RAD = M_PI / 180.0;
            static std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
            std::uniform_real_distribution<double> aRand(-180, 180), pRand(-0.5, 0.5);
            auto a1 = Eigen::AngleAxisd(aRand(engine) * DEG_TO_RAD, Eigen::Vector3d(0, 0, 1));
            auto a2 = Eigen::AngleAxisd(aRand(engine) * DEG_TO_RAD, Eigen::Vector3d(0, 1, 0));
            auto a3 = Eigen::AngleAxisd(aRand(engine) * DEG_TO_RAD, Eigen::Vector3d(1, 0, 0));
            return {(a1 * a2 * a3).toRotationMatrix(), Eigen::Vector3d(pRand(engine), pRand(engine), pRand(engine))};
        }

        static auto ExtractRange(const std::vector<IMUFrame::Ptr> &data, double st, double et) {
            auto sIter = std::find_if(data.begin(), data.end(), [st](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() > st;
            });
            auto eIter = std::find_if(data.rbegin(), data.rend(), [et](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() < et;
            }).base();
            return std::pair(sIter, eIter);
        }

        static void foo(ceres::Problem *prob, Eigen::Vector3d *POS_BiInBr, Eigen::Vector3d *GRAVITY,
                        const Eigen::Vector3d &bVec, const Eigen::Matrix3d &AMat,
                        Eigen::Vector3d *sVel, Eigen::Vector3d *eVel, double dt) {
            auto costFunc = VelIntegrationFactor::Create(bVec, AMat, dt, 1.0);
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

            paramBlockVec.push_back(POS_BiInBr->data());

            paramBlockVec.push_back(GRAVITY->data());

            paramBlockVec.push_back(sVel->data());
            paramBlockVec.push_back(eVel->data());

            prob->AddResidualBlock(costFunc, nullptr, paramBlockVec);
            prob->SetManifold(GRAVITY->data(), new ceres::SphereManifold<3>());

            prob->SetParameterBlockConstant(sVel->data());
            prob->SetParameterBlockConstant(eVel->data());
        }

        static void Learn() {
            auto trajBrToBr0 = *ns_ctraj::SimuWaveMotion2<4>(2.0, 0.5, 0.0, 2 * M_PI, 1000.0).GetTrajectory();
            auto SE3_BiToBr = GeneratePoseBias();
            auto trajBiToBr0 = trajBrToBr0 * SE3_BiToBr;
            Eigen::Vector3d gravity(0.0, 0.0, -9.8);
            auto imuMes = trajBiToBr0.ComputeIMUMeasurement(gravity, 1.0 / 1000.0);
            // for (const auto &item: imuMes) {
            //     double t = item->GetTimestamp();
            //     Eigen::Vector3d V3 = trajBrToBr0.LinearAcceInRef(t) - trajBiToBr0.LinearAcceInRef(t);
            //     Eigen::Matrix3d m1 = Sophus::SO3d::hat(trajBrToBr0.AngularVeloInRef(t));
            //     Eigen::Matrix3d m2 = Sophus::SO3d::hat(trajBrToBr0.AngularAcceInRef(t));
            //
            //     Eigen::Vector3d V4 = -(m2 + m1 * m1) * (trajBrToBr0.Pose(t).so3() * SE3_BiToBr.translation());
            //
            //     std::cout << V3.transpose() << std::endl;
            //     std::cout << V4.transpose() << std::endl;
            //
            //     Eigen::Vector3d V5 = V3 + trajBrToBr0.Pose(t).so3() * SE3_BiToBr.so3() * item->GetAcce();
            //     Eigen::Vector3d V6 = trajBrToBr0.LinearAcceInRef(t) - gravity;
            //     std::cout << V5.transpose() << std::endl;
            //     std::cout << V6.transpose() << std::endl << std::endl;
            //
            //     std::cin.get();
            // }
            auto prob = ceres::Problem();
            double st = trajBrToBr0.MinTime() + 0.5, et = trajBrToBr0.MaxTime() - 0.5, dt = 0.05;

            std::vector<Eigen::Vector3d> linVelSeq(std::floor((et - st) / dt), Eigen::Vector3d::Zero());

            for (int i = 0; i < static_cast<int>(linVelSeq.size()) - 1; ++i) {
                int sIdx = i, eIdx = i + 1;
                Eigen::Vector3d *sVel = &linVelSeq.at(sIdx), *eVel = &linVelSeq.at(eIdx);

                // extract data by considering the initialized time offsets
                double sTime = sIdx * dt + st, eTime = eIdx * dt + st;
                auto [sIter, eIter] = ExtractRange(imuMes, sTime, eTime);

                *sVel = trajBrToBr0.LinearVeloInRef(sTime);
                *eVel = trajBrToBr0.LinearVeloInRef(eTime);

                // vector and matrix sequence for integration
                std::vector<std::pair<double, Eigen::Vector3d>> bVecSeq;
                std::vector<std::pair<double, Eigen::Matrix3d>> AMatSeq;

                auto SO3_BiToBr = SE3_BiToBr.so3();

                for (auto iter = sIter; iter != eIter; ++iter) {
                    const auto &frame = *iter;
                    double curTime = frame->GetTimestamp();

                    auto SO3_BrToBr0 = trajBrToBr0.Pose(curTime).so3();

                    // angular velocity in world
                    auto SO3_VEL_BrToBr0InBr0 = trajBrToBr0.AngularVeloInRef(curTime);
                    Eigen::Matrix3d SO3_VEL_MAT = Sophus::SO3d::hat(SO3_VEL_BrToBr0InBr0);

                    // angular acceleration in world
                    auto SO3_ACCE_BrToBr0InBr0 = trajBrToBr0.AngularAcceInRef(curTime);
                    Eigen::Matrix3d SO3_ACCE_MAT = Sophus::SO3d::hat(SO3_ACCE_BrToBr0InBr0);

                    // store
                    bVecSeq.emplace_back(curTime, SO3_BrToBr0 * SO3_BiToBr * frame->GetAcce());
                    AMatSeq.emplace_back(curTime,
                                         (SO3_ACCE_MAT + SO3_VEL_MAT * SO3_VEL_MAT) * SO3_BrToBr0.matrix());
                }

                // integration
                Eigen::Vector3d bVec = TrapIntegrationOnce(bVecSeq);
                Eigen::Matrix3d AMat = TrapIntegrationOnce(AMatSeq);

                foo(&prob, &SE3_BiToBr.translation(), &gravity, bVec, AMat, sVel, eVel, eTime - sTime);

                // Eigen::Vector3d V1 = bVec - AMat * SE3_BiToBr.translation();
                // Eigen::Vector3d V2 = trajBrToBr0.LinearVeloInRef(eTime) - trajBrToBr0.LinearVeloInRef(sTime) -
                //                      gravity * (eTime - sTime);
                // std::cout << V1.transpose() << std::endl;
                // std::cout << V2.transpose() << std::endl;
                // Eigen::Vector3d residuals = (bVec - AMat * SE3_BiToBr.translation())
                //                             - (*eVel - *sVel - gravity * (eTime - sTime));
                // std::cout << residuals.transpose() << std::endl;
                // std::cin.get();
            }
            ceres::Solver::Summary summary;
            ceres::Solver::Options options = Estimator::DefaultSolverOptions();
            // prob.SetParameterBlockConstant(SE3_BiToBr.translation().data());
            // prob.SetParameterBlockConstant(gravity.data());

            std::cout << SE3_BiToBr.translation().transpose() << std::endl;
            // SE3_BiToBr.translation().setZero();
            // std::cout << SE3_BiToBr.translation().transpose() << std::endl;
            std::cout << gravity.transpose() << std::endl;

            ceres::Solve(options, &prob, &summary);

            std::cout << SE3_BiToBr.translation().transpose() << std::endl;
            std::cout << gravity.transpose() << std::endl;
        }
    };
}

#endif //MI_CALIB_VELOCITY_INTEGRATION_HPP
