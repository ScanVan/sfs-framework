# Project Overview

This framework implements an experimental structure from motion pipeline based on _true_ spherical images.

# Copyright and License

**sfs-framework** - Nils Hamel, Charles Papon <br >
Copyright (c) 2019 DHLAB, EPFL & HES-SO Valais-Wallis

This program is licensed under the terms of the GNU GPLv3. Documentation and illustrations are licensed under the terms of the CC BY 4.0.

# Dependencies

The _sfs-framework_ is developed on Ubuntu 16.40 LTS and comes with the following package dependencies :

* build-essential
* cmake
* libeigen3-dev

and the following external dependencies :

* opencv 4.0.1
* yaml-cpp

The code documentation is built using doxygen. See [COPYRIGHT](COPYRIGHT.md) file for third parties redistribution.

## OpenCV

```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install build-essential libgtk2.0-dev libjpeg-dev libtiff5-dev libjasper-dev libopenexr-dev cmake python-dev python-numpy python-tk libtbb-dev libeigen3-dev yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev libqt4-dev libqt4-opengl-dev sphinx-common texlive-latex-extra libv4l-dev libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev default-jdk ant libvtk5-qt4-dev
cd ~
mkdir opencv
cd opencv
wget https://github.com/opencv/opencv/archive/4.0.1.tar.gz
tar -xvzf 4.0.1.tar.gz
wget https://github.com/opencv/opencv_contrib/archive/4.0.1.zip
unzip 4.0.1.zip
cd opencv-4.0.1
mkdir build
cd build
cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON .. -DCMAKE_BUILD_TYPE=RELEASE -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.0.1/modules ..
make -j$(nproc)
sudo make install
echo '/usr/local/lib' | sudo tee --append /etc/ld.so.conf.d/opencv.conf
sudo ldconfig
echo 'PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig' | sudo tee --append ~/.bashrc
echo 'export PKG_CONFIG_PATH' | sudo tee --append ~/.bashrc
source ~/.bashrc
```

## yaml-cpp

```
wget https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.6.3.tar.gz
tar xvzf yaml-cpp-0.6.3.tar.gz
cd yaml-cpp-yaml-cpp-0.6.3
mkdir build
cd build
cmake .. 
make -j$(nproc)
sudo make install
```


# Usage

```bash
mkdir build
cd build
cmake ..
make
bin/sfs-framework CONFIG_YAML_FILE
```

# Eclipse debug

```bash
cd ..
mkdir sfs-framework.git-debug
cd sfs-framework.git-debug
cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j$(nproc) ../sfs-framework.git
```


