# Overview

_Note : This framework is under active developments_

This framework implements an experimental structure from motion pipeline designed for _true_ spherical image. From an sequence of _true_ spherical images, the pipeline is designed to take advantage of the specificity of such images to compute the 3D structure of the environment captured on the images. It implement a novel optimization algorithm that is able to deduce the pose of the image based on the matching features.

The considered _true_ spherical images are characterized by the centrality of the camera that captured them. A camera able to capture _true_ spherical is then understood as a camera that focus all rays of light on the same and unique focal point, producing parallax-free panoramic images.

# ScanVan Project

The _ScanVan Project_ was funded by the Swiss National Science Foundation (SNF, PNR 76 Big Data) and won by the _DHLAB_ of _EPFL_ and the _HES-SO Valais Wallis_. The goal of the project was to demonstrate the possibility of city mass and continuous digitization using central spherical cameras. Using such device allows to simplify the 3D structures computation and to drastically decrease the complexity of images acquisition.

The _EPFL_ team realized the theoretical design of the camera and defined a new pose estimation algorithm able to be applied to the case of images acquired by such device. The _HES-SO Valais Wallis_ were in charge of translating the theoretic design of the camera into a physical device achieving the centrality. The codes implementation, validation and testing was shared between the two teams.

Summary of the _ScanVan Project_ codes :

* [Camera calibration](https://github.com/ScanVan/Calibration-CPP)
* [Camera images acquisition](https://github.com/ScanVan/CameraImageAcquisition-CPP)
* [Camera images debayering](https://github.com/ScanVan/ConvertRawToBmp)
* [Panoramic images computation](https://github.com/ScanVan/Equirectangular-CPP)
* [Structure from spheres pipeline](https://github.com/ScanVan/sfs-framework)

These codes and the physical camera give access to a full city digitization pipeline. Other codes were implemented during the research phase and can be found [here](https://github.com/ScanVan).

# sfs-framework

To illustrate the pipeline results, the following spherical image are used. They are produced by the spherical central camera built during the _ScanVan Project_ that is able to produce _true_ spherical images. The following images shows example of such images in equirectangular projection :

<br />
<p align="center">
<img src="https://github.com/ScanVan/sfs-framework/blob/master/doc/20200220-155121-476244.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/ScanVan/sfs-framework/blob/master/doc/20200220-155121-726279.jpg?raw=true" width="384">
<br />
<i>Example of central spherical camera images sequence</i>
</p>
<br />

As the camera acquire almost everything surrounding it, it also capture part of itself and the vehicle on which it is mounted. This parts have been hidden on the previous images and replaced by extrapolated pixels.

The pipeline starts by computing a sparse model using traditional features (_AKAZE_) in small amount. This allows to compute the odometry of the camera. In order to improve the performances, the pipeline assumes that the images are acquired in continuous sequence. It is also able to remove _duplicated images_ in order to take into account the effect of traffic and traffic lights during image acquisition.

The following images gives an example of a computed odometry and its sparse model on a small sequence of images :

<br />
<p align="center">
<img src="https://github.com/ScanVan/sfs-framework/blob/master/doc/sparse-1.png?raw=true" width="384">
&nbsp;
<img src="https://github.com/ScanVan/sfs-framework/blob/master/doc/sparse-2.png?raw=true" width="384">
<br />
<i>Example of sparse model obtain with a 20 image sequence</i>
</p>
<br />

The blue dots are associated to the computed camera position while the other points are colored according to the quality of the constraint that led to their placement in the 3D space.

Based on the computed odometry, the pipeline is able to produce dense model using _optical flow_ to derive dense matching between the connected images. As the odometry is known, this allows to place the dense matches in the 3D space with limited computation. The following images gives an illustration of the obtained result on the previous example :

<br />
<p align="center">
<img src="https://github.com/ScanVan/sfs-framework/blob/master/doc/dense-1.png?raw=true" width="384">
&nbsp;
<img src="https://github.com/ScanVan/sfs-framework/blob/master/doc/dense-2.png?raw=true" width="384">
<br />
<i>Example of dense model obtain with a 20 image sequence</i>
</p>
<br />

The color are assigned in the same way as for the sparse model.

The _following page (in preparation)_ gives you the detailed instructions to use the pipeline on a set of spherical images.

# Copyright and License

**sfs-framework** - Nils Hamel, Charles Papon <br >
Copyright (c) 2019-2020 DHLAB, EPFL & HES-SO Valais-Wallis

This program is licensed under the terms of the GNU GPLv3. Documentation and illustrations are licensed under the terms of the CC BY 4.0.

See [COPYRIGHT](COPYRIGHT.md) file for third parties redistribution.

# Dependencies

The _sfs-framework_ comes with the following package (Ubuntu 16.04 LTS) dependencies ([Instructions](DEPEND.md)) :


* build-essential
* cmake
* libeigen3-dev

and the following external dependencies ([Instructions](DEPEND.md)) :

* opencv 4.0.1 
* yaml-cpp 0.6.3

The code documentation is built using Doxygen.

# Compilation

To build the project in the cloned directory, use the following commands :

```bash
mkdir Build
cd Build
cmake . ..
make -j [ncores]
```
