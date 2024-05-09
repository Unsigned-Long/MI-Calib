// Copyright (c) 2023. Created on 7/7/23 1:31 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include "config/configor.h"
#include "cereal/archives/xml.hpp"
#include "fstream"
#include "spdlog/spdlog.h"
#include "calib/status.hpp"

namespace ns_mi {
    // ------------------------
    // static initialized filed
    // ------------------------
    Configor::DataStream Configor::dataStream = {};
    Configor::Prior Configor::prior = {};
    Configor::Prior::Weight Configor::Prior::weight = {};
    Configor::Preference Configor::preference = {};

    std::map<std::string, Configor::DataStream::IMUConfig> Configor::DataStream::IMUTopics = {};
    std::string Configor::DataStream::ReferIMU = {};
    std::string Configor::DataStream::BagPath = {};
    double Configor::DataStream::BeginTime = {};
    double Configor::DataStream::Duration = {};
    std::string Configor::DataStream::OutputPath = {};

    double Configor::Prior::GravityNorm = {};
    double Configor::Prior::TimeOffsetPadding = {};
    double Configor::Prior::KnotTimeDist::SO3Spline = {};
    double Configor::Prior::KnotTimeDist::LinAcceSpline = {};

    double Configor::Prior::Weight::AcceWeight = {};
    double Configor::Prior::Weight::GyroWeight = {};

    bool Configor::Preference::UseCudaInSolving = {};
    bool Configor::Preference::OutputParamInEachIter = {};
    bool Configor::Preference::OutputBSplines = {};
    bool Configor::Preference::OutputKinematics = {};
    CerealArchiveType::Enum Configor::Preference::OutputDataFormat = CerealArchiveType::Enum::YAML;
    const std::map<CerealArchiveType::Enum, std::string> Configor::Preference::FileExtension = {
            {CerealArchiveType::Enum::YAML,   ".yaml"},
            {CerealArchiveType::Enum::JSON,   ".json"},
            {CerealArchiveType::Enum::XML,    ".xml"},
            {CerealArchiveType::Enum::BINARY, ".bin"}
    };
    bool Configor::Preference::OptTemporalParams = {};
    int Configor::Preference::ThreadsToUse = {};
    const std::string Configor::Preference::SO3_SPLINE = "SO3_SPLINE";
    const std::string Configor::Preference::ACCE_SPLINE = "ACCE_SPLINE";
    double Configor::Preference::ScaleSpline = {};
    double Configor::Preference::ScaleCoord = {};

    Configor::Configor() = default;

    void Configor::PrintMainFields() {
        std::stringstream streamIMUTopicModel;
        for (const auto &[imuTopic, config]: DataStream::IMUTopics) {
            streamIMUTopicModel << imuTopic << ": " << config.Type << " ";
        }

        std::string IMUTopicModel = streamIMUTopicModel.str();

#define DESC_FIELD(field) #field, field
#define DESC_FORMAT "\n{:>35}: {}"
        spdlog::info(
                "main fields of configor:"
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT,
                DESC_FIELD(DataStream::ReferIMU),
                DESC_FIELD(DataStream::BagPath),
                DESC_FIELD(IMUTopicModel),
                DESC_FIELD(DataStream::BeginTime),
                DESC_FIELD(DataStream::Duration),
                DESC_FIELD(DataStream::OutputPath),
                DESC_FIELD(Prior::GravityNorm),
                DESC_FIELD(Prior::TimeOffsetPadding),
                DESC_FIELD(Prior::KnotTimeDist::SO3Spline),
                DESC_FIELD(Prior::KnotTimeDist::LinAcceSpline),
                DESC_FIELD(Prior::Weight::AcceWeight),
                DESC_FIELD(Prior::Weight::GyroWeight),
                DESC_FIELD(Preference::UseCudaInSolving),
                DESC_FIELD(Preference::OutputParamInEachIter),
                DESC_FIELD(Preference::OutputBSplines),
                DESC_FIELD(Preference::OutputKinematics),
                "Preference::OutputDataFormat", EnumCast::enumToString(Preference::OutputDataFormat),
                DESC_FIELD(Preference::OptTemporalParams),
                DESC_FIELD(Preference::ThreadsToUse)
        );

#undef DESC_FIELD
#undef DESC_FORMAT
    }

    void Configor::CheckConfigure() {
        if (DataStream::IMUTopics.size() < 2) {
            throw Status(
                    Status::Flag::ERROR, "the imu topic num (i.e., DataStream::IMUTopic) should be larger equal than 2!"
            );
        }
        for (const auto &imuTopic: DataStream::IMUTopics) {
            if (imuTopic.first.empty()) {
                throw Status(Status::Flag::ERROR, "empty imu topic exists!");
            }
        }
        if (DataStream::IMUTopics.find(DataStream::ReferIMU) == DataStream::IMUTopics.cend()) {
            throw Status(Status::Flag::ERROR, "the reference IMU is not set, it should be one of the IMUs!");
        }
        if (DataStream::BagPath.empty()) {
            throw Status(Status::Flag::ERROR, "the ros bag path (i.e., DataStream::BagPath) is empty!");
        }
        if (DataStream::OutputPath.empty()) {
            throw Status(Status::Flag::ERROR, "the output path (i.e., DataStream::OutputPath) is empty!");
        }
        if (Preference::ScaleSpline <= 0.0) {
            throw Status(Status::Flag::ERROR, "the scale of splines in visualization should be positive!");
        }
        if (Preference::ScaleCoord <= 0.0) {
            throw Status(Status::Flag::ERROR, "the scale of coordinates in visualization should be positive!");
        }
        if (Prior::TimeOffsetPadding <= 0.0) {
            throw Status(
                    Status::Flag::ERROR,
                    "the time offset padding (i.e., Prior::TimeOffsetPadding) should be positive!"
            );
        }
    }

    Configor::Ptr Configor::Create() {
        return std::make_shared<Configor>();
    }

    std::string Configor::GetFormatExtension() {
        return Preference::FileExtension.at(Preference::OutputDataFormat);
    }

    bool Configor::LoadConfigure(const std::string &filename, CerealArchiveType::Enum archiveType) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        auto archive = GetInputArchiveVariant(file, archiveType);
        auto configor = Configor::Create();
        SerializeByInputArchiveVariant(archive, archiveType, cereal::make_nvp("Configor", *configor));
        configor->CheckConfigure();
        return true;
    }

    bool Configor::SaveConfigure(const std::string &filename, CerealArchiveType::Enum archiveType) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        auto archive = GetOutputArchiveVariant(file, archiveType);
        SerializeByOutputArchiveVariant(archive, archiveType, cereal::make_nvp("Configor", *this));
        return true;
    }
}
