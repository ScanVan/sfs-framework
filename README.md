# Overview

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

As an example of the _sfs-framework_ usage, a small set of _spherical images_ is used. These images are produced by the spherical central camera built during the _ScanVan project_. The following illustrations show a summary of the images used for this example :

<br />
<p align="center">
<img src="https://github.com/ScanVan/sfs-framework/blob/master/doc/20190319-103833-344893.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/ScanVan/sfs-framework/blob/master/doc/20190319-103833-594895.jpg?raw=true" width="384">
<br />
<i>Illustration of the spherical images dataset</i>
</p>
<br />

The spherical images dataset along with the input configuration and pipeline outputs are accessible in [this repository](empty) as a result of this usage example.

As a first step, a directories structure is created for the images, the mask and the pipeline outputs :

    mkdir -p ~/sfs-framework-example/image
    mkdir -p ~/sfs-framework-example/output

Place the spherical images in the _image_ directory. The pipeline uses the alphabetical order to import the images.

As the camera acquire almost everything surrounding it, it also capture part of itself and the vehicle on which it is mounted. This is the reason a mask image is provided to the pipeline to indicates the pixel areas to drop during reconstruction process. In the example, the mask is placed as follows :

    ~/sfs-framework-example/mask.png

The configuration of the pipeline for this set of image is provided through a _YAML_ file that is placed as follows :

    ~/sfs-framework-example/config.yaml

The next step consists in computed the odometry of the images which produces a sparse model. The computation of the odometry is made to compute the relative rotation and translation that appears between the image. The frame in which this position and orientation are expressed is the frame of the first image. To start the odometry computation, the following command can be used :

    bin/sfs-framework ~/sfs-framework-example/config.yaml 2>&1 | tee -a ~/sfs-framework-example/output/logs

which allows to print everything on the terminal and in the provided log file.

At the end of the odometry computation process, the following sparse model should be obtained :

<br />
<p align="center">
<img src="https://github.com/ScanVan/sfs-framework/blob/master/doc/sparse-1.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/ScanVan/sfs-framework/blob/master/doc/sparse-2.jpg?raw=true" width="384">
<br />
<i>Example of sparse model obtain with a sequence of five spherical images</i>
</p>
<br />


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
