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


## Citation
Please cite the following paper:
```
Guidolin, M., Tagliapietra, L., Menegatti, E., & Reggiani, M. (2023). Hi-ROS: Open-source multi-camera sensor fusion for real-time people tracking. Computer Vision and Image Understanding, 232, 103694.
```

Bib citation source:
```bibtex
@article{GUIDOLIN2023103694,
  title = {Hi-ROS: Open-source multi-camera sensor fusion for real-time people tracking},
  journal = {Computer Vision and Image Understanding},
  volume = {232},
  pages = {103694},
  year = {2023},
  issn = {1077-3142},
  doi = {https://doi.org/10.1016/j.cviu.2023.103694},
  url = {https://www.sciencedirect.com/science/article/pii/S1077314223000747},
  author = {Mattia Guidolin and Luca Tagliapietra and Emanuele Menegatti and Monica Reggiani},
  keywords = {Markerless motion capture, Multi-view body tracking, Real-time, ROS}
}
```
