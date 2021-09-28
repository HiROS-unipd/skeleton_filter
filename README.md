# skeleton_filter


## Dependencies
* [RTB Filter](https://github.com/RealTimeBiomechanics/Filter)
* [skeleton\_msgs](https://gitlab.com/hi-ros/skeleton_msgs)


## Launch files
**default.launch**
Contains the default values for each parameter

**custom\_configuration\_example.launch**
Contains an example on how to set some parameters of choice


## Usage
This ROS package takes as input a skeleton\_msgs::SkeletonGroup and perform low-pass filtering of each skeleton's poses.

```
roslaunch hiros_skeleton_filter custom_configuration_example.launch
```


## Parameters

| Parameter          | Description                                                                     |
| ------------------ | ------------------------------------------------------------------------------- |
| `node_required`    | Set if the other ROS nodes on the PC should be killed when the driver is killed |
| `node_name`        | Node name                                                                       |
| `input_topic`      | Topic containing the input skeletons                                            |
| `output_topic`     | Topic that will be published containing the filtered skeletons                  |
| `cutoff_frequency` | Cutoff frequency of the filter                                                  |

