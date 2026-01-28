# Kalibr Camera-IMU Calibration

## Prerequisites

Target can be created using
```sh
rosrun kalibr kalibr_create_target_pdf --type apriltag --nx 6 --ny 6 --tsize 0.08 --tspace 0.3 --padding 0.08
```

Create target.yaml if not already present in recording directory, eg
```yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.08 # size of one tag side in meters
tagSpacing: 0.3 # ratio of tag spacing to tag size
codeOffset: 0
```

Create imu.yaml if not already present in recording directory, eg
```yaml
#Accelerometer - relaxed noise parameters for better camera fit
accelerometer_noise_density: 0.05          # [m/s^2/sqrt(Hz)]   (accel "white noise")
accelerometer_random_walk: 0.001           # [m/s^3/sqrt(Hz)]   (accel bias diffusion)

#Gyroscope - relaxed noise parameters for better camera fit  
gyroscope_noise_density: 0.005             # [rad/s/sqrt(Hz)]   (gyro "white noise")
gyroscope_random_walk: 0.0001              # [rad/s^2/sqrt(Hz)] (gyro bias diffusion)

rostopic: /imu0
update_rate: 200.0                         # [Hz]
```

Note: Lower noise values = more trust in IMU = higher reprojection error.
If reprojection error is too high, increase noise density values.

## Step 1: Camera Intrinsics Calibration

Record a **slow, steady** dataset for camera intrinsics:
- Move camera slowly to avoid motion blur
- Keep the entire AprilGrid visible in frame
- Capture target at various angles and distances
- ~100-200 frames is sufficient

```sh
# set recording directory for intrinsics calibration
INTRINSICS_DIR=/workspaces/kalibr/data/be2_15fps_aprilgrid_intrinsics

# convert TUM dataset to ROS bag (--imu-signs for Orbbec: flip gyro signs)
python3 /workspaces/kalibr/aslam_offline_calibration/kalibr/python/kalibr_bagcreater_tum \
    --folder $INTRINSICS_DIR \
    --output-bag $INTRINSICS_DIR/recording.bag \
    --imu-signs "1,1,1,-1,-1,-1"

# run camera calibration
rosrun kalibr kalibr_calibrate_cameras \
    --bag $INTRINSICS_DIR/recording.bag \
    --topics /cam0/image_raw \
    --target $INTRINSICS_DIR/target.yaml \
    --models pinhole-radtan \
    --dont-show-report
```

**Output:** `$INTRINSICS_DIR/recording-camchain.yaml` (camera intrinsics, used in Step 2)

Expected reprojection error: < 0.5 px

## Step 2: IMU-Camera Calibration

Record a **dynamic** dataset for IMU-camera extrinsics:
- Excite all 6 degrees of freedom (rotate AND translate)
- Smooth but varied motion
- Keep AprilGrid visible as much as possible
- ~60 seconds is sufficient

```sh
# set recording directory for IMU-camera calibration
IMUCAM_DIR=/workspaces/kalibr/data/be2_15fps_aprilgrid5

# convert TUM dataset to ROS bag
python3 /workspaces/kalibr/aslam_offline_calibration/kalibr/python/kalibr_bagcreater_tum \
    --folder $IMUCAM_DIR \
    --output-bag $IMUCAM_DIR/recording.bag \
    --imu-signs "1,1,1,-1,-1,-1"

# run IMU-camera calibration using intrinsics from Step 1
rosrun kalibr kalibr_calibrate_imu_camera \
    --bag $IMUCAM_DIR/recording.bag \
    --cam $INTRINSICS_DIR/recording-camchain.yaml \
    --imu $IMUCAM_DIR/imu.yaml \
    --target $IMUCAM_DIR/target.yaml \
    --dont-show-report
```

Expected errors:
- Reprojection: < 1.5 px
- Gyroscope: < 0.15 rad/s
- Accelerometer: < 1.0 m/s²

**Output:** `$IMUCAM_DIR/recording-camchain-imucam.yaml` (IMU-camera extrinsics and timeshift)

## Step 3: Export to RMAP Format

Convert Kalibr output to RMAP format for VIO:

```sh
python3 /workspaces/kalibr/aslam_offline_calibration/kalibr/python/kalibr_export_vio \
    --camchain $IMUCAM_DIR/recording-camchain-imucam.yaml \
    --imu $IMUCAM_DIR/imu.yaml \
    --results $IMUCAM_DIR/recording-results-imucam.txt \
    --output $IMUCAM_DIR/rmap_calibration.yaml
```

**Output:** `$IMUCAM_DIR/rmap_calibration.yaml` with camera intrinsics, IMU noise, `camera_T_imu` transform, timeshift, and gravity magnitude
