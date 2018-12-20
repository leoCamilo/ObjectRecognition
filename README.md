# ObjectRecognition

An object recognition system implemented using [ROS](http://www.ros.org/) and [PCL](http://pointclouds.org/), using a Kinect sensor with point cloud data.

The system uses the ROS approach of [nodes](http://wiki.ros.org/Nodes) to implement the functionalities, nodes implemented in the system:
  - [ransac](#ransac) (floor removing)
  - [downsample](#downsample) (sample reducer using voxelgrid)
  - [clustering](#clustering) (object segmentation using euclidian distance)
  - [association](#association) (object temporal association using KNN)
  - [vfh-tracker](#vfh-tracker) (object classification using VFH and KNN)
  - [snapshot](/kinect/src/nodes/take-snapshot.cpp) (creator of models)
  - [viewer](/kinect/src/nodes/pcl-viewer.cpp) (object viewer)
  - [box-tracker](/kinect/src/nodes/prediction-tracking.cpp) (boundary box viewer)

## initial

For this work the ROS is needed, to install ros following [this steps](http://wiki.ros.org/kinetic/Installation). the [openni package](http://wiki.ros.org/openni_kinect) is needed to read Kinect data. The installation tutorial can be found on the package page.

For run openni ``` roslaunch openni_launch openni.launch ```, this launcher initialize the ```roscore``` and the openni nodes. For data visualization, the [rviz](http://wiki.ros.org/rviz) package can be used. ```rosrun rviz rviz```

## ransac

the [ransac node](/kinect/src/nodes/ransac.cpp) apply the [ransac approach](https://en.wikipedia.org/wiki/Random_sample_consensus) for remove the floor associated points in the cloud.

command: ``` rosrun kinect ransac [-i] ``` the ``` -i ``` optional paramether is an inverter, for return only the floor points.

<p align="center">
  <img src="/screenshots/Screenshot%20from%202017-05-10%2000:49:10.png" width="45%">
  <img src="/screenshots/Screenshot%20from%202017-05-10%2000:19:54.png" width="45%">
</p>

## downsample

the [downsample node](/kinect/src/nodes/voxel-downsample.cpp) use the [VoxelGrid](http://pointclouds.org/documentation/tutorials/voxel_grid.php) approach implemented on PCL. With a 3 cm voxel size.

command: ``` rosrun kinect downsample ```

<p align="center">
  <img src="/screenshots/Screenshot%20from%202017-05-11%2000:01:05.png" width="45%">
  <img src="/screenshots/Screenshot%20from%202017-05-11%2000:01:15.png" width="45%">
  <img src="/screenshots/Screenshot%20from%202017-05-11%2000:08:34.png" width="45%">
  <img src="/screenshots/Screenshot%20from%202017-05-11%2000:08:46.png" width="45%">
</p>

## clustering

the [clustering node](/kinect/src/nodes/euclidian-clustering.cpp) use the [Euclidian distance](http://pointclouds.org/documentation/tutorials/cluster_extraction.php) approach implemented on PCL too. Recieves a dowmsampled cloud, and returns in different channels each object segmentated on scene. The limit of maximum clouds are set to 10, the clouds has a limit between 50 and 1000 points, with a distance threshold of 15 cm.

command: ``` rosrun kinect clustering ```

<p align="center">
  <img src="/screenshots/Screenshot%20from%202017-05-26%2018:05:02.png" width="45%">
  <img src="/screenshots/Screenshot%20from%202017-05-26%2018:11:26.png" width="45%">
</p>

## association

the [association](/kinect/src/nodes/knn-association.cpp) module use the [K-nearest-neighbors](https://en.wikipedia.org/wiki/K-nearest_neighbors_algorithm) approach, each segment found on clustering step is associated through the time.

command: ``` rosrun kinect association ```

<p align="center">
  <img src="/screenshots/Screenshot%20from%202017-05-30%2016:19:05.png" width="45%">
  <img src="/screenshots/Screenshot%20from%202017-05-30%2016:19:22.png" width="45%">
</p>

## vfh-tracker

this module use the descriptor [VFH](http://pointclouds.org/documentation/tutorials/vfh_estimation.php) found on PCL to create a feature histogram for each object associated, and a KNN aproach to classify the histogram, the chi square distance is used to get the distance between the histograms. To this node works is needed a training set, with the model of the object you want to track. As result, a red boundary box is set on scene to show the tracked object.

command: ``` rosrun kinect vfh-tracker '\model\path\folder' ```

<p align="center">
  <img src="/screenshots/Screenshot%20from%202017-06-21%2022:12:04.png" width="45%">
  <img src="/screenshots/Screenshot%20from%202017-06-21%2022:12:36.png" width="45%">
  <img src="/screenshots/Screenshot%20from%202017-06-21%2022:13:29.png" width="45%">
  <img src="/screenshots/Screenshot%20from%202017-06-21%2022:14:06.png" width="45%">
  <img src="/screenshots/Screenshot%20from%202017-06-21%2022:14:26.png" width="45%">
  <img src="/screenshots/Screenshot%20from%202017-06-21%2022:14:42.png" width="45%">
</p>

## videos

- [video 1](https://youtu.be/uHN-OzuG_P0)
- [video 2](https://youtu.be/6RJNfO7ljw0)
- [video 3](https://youtu.be/UaJ_SajaMf0)

## paper
  [here](/paper/tcc.pdf)
