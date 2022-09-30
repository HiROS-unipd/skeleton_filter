# Hi-ROS Skeleton Filter

This ROS package takes as input a skeleton group and performs low-pass filtering of each skeleton's poses.


## Dependencies
* [RTB Filter](https://github.com/RealTimeBiomechanics/Filter)
* [Hi-ROS Skeleton Messages](https://github.com/hiros-unipd/skeleton_msgs)


## Parameters

| Parameter           | Description                                                                   |
| ------------------- | ----------------------------------------------------------------------------- |
| `input_topic`       | Topic containing the input skeletons                                          |
| `output_topic`      | Topic that will be published containing the filtered skeletons                |
| `filter`            | Filter to use (`statespace` or `butterworth`)                                 |
| `butterworth_order` | Order of the Butterworth filter                                               |
| `sample_frequency`  | Input data sample frequency (only required for Butterworth filter)            |
| `cutoff_frequency`  | Cutoff frequency of the filter                                                |


## Usage
```
ros2 launch hiros_skeleton_filter default.launch.py
```
