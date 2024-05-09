// Copyright (c) 2023. Created on 9/20/23 8:06 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef MI_CALIB_IMU_DATA_LOADER_H
#define MI_CALIB_IMU_DATA_LOADER_H

#include "sensor_msgs/Imu.h"
#include "sbg_driver/SbgImuData.h"
#include "rosbag/message_instance.h"
#include "sensor/imu.h"

namespace ns_mi {
    enum class IMUModelType {
        SENSOR_IMU,
        SBG_IMU
    };


    class IMUDataLoader {
    public:
        using Ptr = std::shared_ptr<IMUDataLoader>;

    protected:
        IMUModelType _imuModel;

    public:
        explicit IMUDataLoader(IMUModelType imuModel);

        virtual IMUFrame::Ptr UnpackFrame(const rosbag::MessageInstance &msgInstance) = 0;

        static IMUDataLoader::Ptr GetLoader(const std::string &imuModelStr);

        [[nodiscard]] IMUModelType GetIMUModel() const;
    };

    class SensorIMULoader : public IMUDataLoader {
    public:
        using Ptr = std::shared_ptr<SensorIMULoader>;

    public:
        explicit SensorIMULoader(IMUModelType imuModel);

        static SensorIMULoader::Ptr Create(IMUModelType imuModel);

        IMUFrame::Ptr UnpackFrame(const rosbag::MessageInstance &msgInstance) override;
    };

    class SbgIMULoader : public IMUDataLoader {
    public:
        using Ptr = std::shared_ptr<SbgIMULoader>;

    public:
        explicit SbgIMULoader(IMUModelType imuModel);

        static SbgIMULoader::Ptr Create(IMUModelType imuModel);

        IMUFrame::Ptr UnpackFrame(const rosbag::MessageInstance &msgInstance) override;
    };
}


#endif //MI_CALIB_IMU_DATA_LOADER_H
