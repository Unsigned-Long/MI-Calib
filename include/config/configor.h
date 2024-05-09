// Copyright (c) 2023. Created on 7/7/23 1:29 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef MI_CALIB_CONFIGOR_H
#define MI_CALIB_CONFIGOR_H

#include "memory"
#include "cereal/archives/json.hpp"
#include "cereal/cereal.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/set.hpp"
#include "util/utils.hpp"
#include "util/enum_cast.hpp"
#include "sensor/imu_intrinsic.hpp"

namespace ns_mi {

    struct Configor {
    public:
        using Ptr = std::shared_ptr<Configor>;

    public:

        static struct DataStream {
            struct IMUConfig {
            public:
                std::string Type;
                std::string Intrinsics;

                IMUConfig() = default;

            public:
                template<class Archive>
                void serialize(Archive &ar) {
                    ar(CEREAL_NVP(Type), CEREAL_NVP(Intrinsics));
                }
            };

            static std::map<std::string, IMUConfig> IMUTopics;
            static std::string ReferIMU;

            static std::string BagPath;
            static double BeginTime;
            static double Duration;

            static std::string OutputPath;

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(
                        CEREAL_NVP(IMUTopics), CEREAL_NVP(ReferIMU), CEREAL_NVP(BagPath),
                        CEREAL_NVP(BeginTime), CEREAL_NVP(Duration), CEREAL_NVP(OutputPath)
                );
            }
        } dataStream;

        static struct Prior {
            static double GravityNorm;
            static constexpr int SplineOrder = 4;
            static double TimeOffsetPadding;

            static struct KnotTimeDist {
                static double SO3Spline;
                static double LinAcceSpline;

            public:
                template<class Archive>
                void serialize(Archive &ar) {
                    ar(CEREAL_NVP(SO3Spline), CEREAL_NVP(LinAcceSpline));
                }
            } knotTimeDist;

            static struct Weight {
                static double AcceWeight;
                static double GyroWeight;

            public:
                template<class Archive>
                void serialize(Archive &ar) {
                    ar(CEREAL_NVP(AcceWeight), CEREAL_NVP(GyroWeight));
                }
            } weight;

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(GravityNorm), CEREAL_NVP(TimeOffsetPadding),
                   cereal::make_nvp("KnotTimeDist", knotTimeDist),
                   cereal::make_nvp("Weight", weight));
            }
        } prior;

        static struct Preference {
            static bool UseCudaInSolving;
            static bool OutputParamInEachIter;
            static bool OutputBSplines;
            static bool OutputKinematics;
            static CerealArchiveType::Enum OutputDataFormat;
            const static std::map<CerealArchiveType::Enum, std::string> FileExtension;
            static bool OptTemporalParams;
            static int ThreadsToUse;

            const static std::string SO3_SPLINE, ACCE_SPLINE;

            static double ScaleSpline;
            static double ScaleCoord;

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(
                        CEREAL_NVP(UseCudaInSolving),
                        CEREAL_NVP(OutputParamInEachIter),
                        CEREAL_NVP(OutputBSplines),
                        CEREAL_NVP(OutputKinematics),
                        CEREAL_NVP(OutputDataFormat),
                        CEREAL_NVP(OptTemporalParams),
                        CEREAL_NVP(ThreadsToUse),
                        CEREAL_NVP(ScaleSpline),
                        CEREAL_NVP(ScaleCoord)
                );
            }
        } preference;

    public:
        Configor();

        static Ptr Create();

        // load configure information from file
        template<class CerealArchiveType=CerealArchiveType::YAML>
        static bool LoadConfigure(const std::string &filename) {
            std::ifstream file(filename);
            auto archive = GetInputArchive<CerealArchiveType>(file);
            auto configor = Configor::Create();
            (*archive)(cereal::make_nvp("Configor", *configor));
            configor->CheckConfigure();
            return true;
        }

        // save configure information to file
        template<class CerealArchiveType=CerealArchiveType::YAML>
        bool SaveConfigure(const std::string &filename) {
            std::ofstream file(filename);
            auto archive = GetOutputArchive<CerealArchiveType>(file);
            (*archive)(cereal::make_nvp("Configor", *this));
            return true;
        }

        // load configure information from file
        static bool LoadConfigure(const std::string &filename, CerealArchiveType::Enum archiveType);

        // save configure information to file
        bool SaveConfigure(const std::string &filename, CerealArchiveType::Enum archiveType);

        // print the main fields
        static void PrintMainFields();

        static std::string GetFormatExtension();

    protected:
        // check the input configure
        static void CheckConfigure();

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(cereal::make_nvp("DataStream", dataStream), cereal::make_nvp("Prior", prior),
               cereal::make_nvp("Preference", preference));
        }
    };
}

#endif //MI_CALIB_CONFIGOR_H
