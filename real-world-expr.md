# MI-Calib: Multiple IMUs Spatiotemporal Calibrator

![Static Badge](https://img.shields.io/badge/Calibration-Multiple_Sensors-red) ![Static Badge](https://img.shields.io/badge/Cpp-17-green) ![ ](https://img.shields.io/badge/Multiple-IMUs-blue) ![Static Badge](https://img.shields.io/badge/MI-Calib-red) ![Static Badge](https://img.shields.io/badge/ROS-1.0-green) ![Static Badge](https://img.shields.io/badge/Python-3.0-blue) ![Static Badge](https://img.shields.io/badge/Continuous-Time-red) ![Static Badge](https://img.shields.io/badge/Bspline-Curves-green) ![Static Badge](https://img.shields.io/badge/Spatiotemporal-Calibrator-blue) ![Static Badge](https://img.shields.io/badge/WHU-SGG-red) ![Static Badge](https://img.shields.io/badge/ULong2-Shuolong_Chen-green) ![Static Badge](https://img.shields.io/badge/Wuhan-China-blue)

<div align=center><img src="img/ico.png" width =100%></div>

### 3.2 Real-world Experiments

#### 3.2.1 Real-world Spatiotemporal Calibration for two IMUs

The data of the real-world experiments we conducted are available here:

```latex
# Google Drive
https://drive.google.com/drive/folders/11GTlHyzjSkt6ZXAZs9t6Tmg11Nah2Bx_?usp=sharing
```

There no difference between simulated launch and real-world one, but the information in config file. For better calibration results, you are expected to pre-calibrate the intrinsics of your IIMUs. Follow the following procedure to obtain them.

<div align=center><img src="img/sensor-suite.jpg" width =80%></div>

#### 3.2.2 Stationary Intrinsics Calibration of IMUs

Considering that the weak observability of the intrinsic parameters (i.e., non-orthogonal factors, rotational misalign-
ments, and biases) in the proposed spatiotemporal calibration, they are required pre-calibrated, which can be accomplished in a separate process by introducing additional stationary prior.

The test datasets for intrinsic calibration are also available at the above Google Drive link. The datasets contain multiple rosbags, each one of them is collected when the body is under stationary. Altogether six kinds of poses are considered for better calibration performance.

Download the datasets, and pass their pathes to the configure named as `config-imu-intri-calib.yaml` . The stationary time pieces are required to be provided in this configure file. Then launch the file named as `mi-calib-imu-intri-calib.launch`:

```
roslaunch mi_calib mi-calib-imu-intri-calib.launch
```

The calibration results would be output as a single file, which would be involved in the following spatiotemporal calibration.