# Overview

This framework implements an experimental structure from motion pipeline designed for _true_ spherical image. From an sequence of _true_ spherical images, the pipeline is designed to take advantage of the specificity of such images to compute the 3D structure of the environment captured on the images. It implement a novel optimization algorithm that is able to deduce the pose of the image based on the matching features.

The considered _true_ spherical images are characterized by the centrality of the camera that captured them. A camera able to capture _true_ spherical is then understood as a camera that focus all rays of light on the same and unique focal point, producing parallax-free panoramic images.

# ScanVan Project

The _ScanVan Project_ was funded by the Swiss National Science Foundation (SNF, PNR 76 Big Data) and won by the _DHLAB_ of _EPFL_ and the _HES-SO Valais Wallis_. The goal of the project was to demonstrate the possibility of city mass and continuous digitization using central spherical cameras. Using such device allows to simplify the 3D structures computation and to drastically decrease the complexity of images acquisition.

The _EPFL_ team realized the theoretical design of the camera and defined a new pose estimation algorithm able to be applied to the case of images acquired by such device. The _HES-SO Valais Wallis_ were in charge of translating the theoretic design of the camera into a physical device achieving the centrality. The codes implementation, validation and testing was shared between the two teams.

Summary of the _ScanVan Project_ codes :

    [Camera calibration](https://github.com/ScanVan/Calibration-CPP)

    [Camera images acquisition](https://github.com/ScanVan/CameraImageAcquisition-CPP)

    [Camera images debayering](https://github.com/ScanVan/ConvertRawToBmp)

    [Panoramic image computation](https://github.com/ScanVan/Equirectangular-CPP)

    [Structure from spheres pipeline](https://github.com/ScanVan/sfs-framework)

These codes and the physical camera give access to a full city digitization pipeline.

# Structure from Spheres

# Copyright and License

**sfs-framework** - Nils Hamel, Charles Papon <br >
Copyright (c) 2019-2020 DHLAB, EPFL & HES-SO Valais-Wallis

This program is licensed under the terms of the GNU GPLv3. Documentation and illustrations are licensed under the terms of the CC BY 4.0.

See [COPYRIGHT](COPYRIGHT.md) file for third parties redistribution.

# Dependencies

The _sfs-framework_ comes with the following package dependencies (Ubuntu 16.04 LTS, [Instructions](DEPEND.md)) :

* build-essential
* cmake
* libeigen3-dev

and the following external dependencies (Ubuntu 16.04 LTS, [Instructions](DEPEND.md)) :

* opencv 4.0.1 
* yaml-cpp 0.6.3

The code documentation is built using doxygen.

# Compilation

To build the project in the cloned directory, use the following commands :

```bash
mkdir Build
cd Build
cmake . ..
make -j [ncores]
```
