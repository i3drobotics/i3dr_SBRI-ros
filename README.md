# I3DR SBRI package

ROS control package for i3dr cameras in SBRI project

## Features

- I3DR Phobos Nuclear stereo camera control
- UR10 control for autonomous mapping
- RTABMAP wrapper for estimation of odometry and building room map

## Run

### Setup catkin workspace

```bash
source devel/setup.sh
```

### Run Phobos Nuclear launch file

```bash
roslaunch i3dr_phobos_nuclear phobos_nuclear split_laser:=true
```

Starts capturing stereo images from phobos nuclear and generates 3D data.

For mapping laser is toggled on and off.

When the laser is on, 3D data is generating with assistance from the laser for a more filled out scene.

When the laser is off, image data can be fed to visual SLAM feature matchers that would otherwise be confused by image data with laser.

Images are captured at 2 fps to allow time for longer exposures.

### Run RTABMAP mapping

```bash
roslaunch i3dr_rtabmap standalone_rtabmap.launch
```

Starts rtabmap nodes for mapping using stereo information from phobos nuclear.

Uses Visual SLAM to estimate odometry of camera and align 3D data in map.

### Run UR10 mapping launcher

```bash
roslaunch i3dr_ur10 ur10_auto_map.launch gui:=false
```

Starts UR10 control nodes and creates mapping path for robot.

Start/stop of the mapping can be triggered using the GUI (if enabled with gui:=true), or via services:

- Go to next pose in mapping sequence: rosservice call /i3dr_scan_next_pose {}

- Go to home pose: rosservice call /i3dr_scan_home_pose {}

- Go to fold pose: rosservice call /i3dr_scan_fold_pose {}

- Start/stop continuous mapping sequence: rosservice call /i3dr_scan_continuous {}

- Cancel movement/sequence: rosservice call /i3dr_scan_cancel_pose {}

Will pause RTABMAP while the robot is moving to avoid errors from motion.

## Rosbags

There are launchers provided for recording rosbags so that different techniques can be tested.

## Record rosbag

```bash
roslaunch i3dr_sbri rosbag_sbri_record.launch folder_name:=path/to/bag bag_name:=BAG_NAME.bag
```

Optional arguments:

- Save TF / joint data from UR10: save_ur10_tf:=[true/false]

## Playback rosbag

```bash
roslauch i3dr_sbri rosbag_sbri_play.launch folder_name:=path/to/bag bag_name:=BAG_NAME.bag
```

Optional arguments:

- Generate 3D from stereo images: stereo_match:=[true/false]

- Run RTABMAP using rosbag data: rtabmap:=[true/false]

- Run RVIZ to visualise data and processes: rviz:=[true/false]
