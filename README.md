# ObjectRecognition

An object recognition system implemented using [ROS](http://www.ros.org/) and [PCL](http://pointclouds.org/), using a Kinect sensor with point cloud data.

nodes implemented:
  - [ransac](/kinect/src/nodes/ransac.cpp) (floor removing)
  - [downsample](/kinect/src/nodes/voxel-downsample.cpp) (sample reducer using voxelgrid)
  - [clustering](/kinect/src/nodes/euclidian-clustering.cpp) (object segmentation using euclidian distance)
  - [association](/kinect/src/nodes/knn-association.cpp) (object temporal association using KNN)
  - [vfh-tracker](/kinect/src/nodes/vfh-tracking.cpp) (object classification using VFH and KNN)
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
