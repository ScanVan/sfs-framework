## Packages and external dependencies installation

The following sections give the installation instructions for the required distribution packages and external dependencies for the supported platforms.

## Required distribution packages

### Ubuntu 16.04 LTS

Required distribution packages installation :

```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install build-essential cmake libeigen3-dev

```

## OpenCV 4.0.1

### Ubuntu 16.04 LTS

This procedure creates a _Build_ directory in the user home directory to perform the compilation.

```
mkdir ~/Build
cd ~/Build
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

After installation, the _Build_ directory can be removed.

## yaml-cpp 0.6.3

### Ubuntu 16.04 LTS

This procedure creates a _Build_ directory in the user home directory to perform the compilation.

```
mkdir ~/Build
cd ~/Build
wget https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.6.3.tar.gz
tar xvzf yaml-cpp-0.6.3.tar.gz
cd yaml-cpp-yaml-cpp-0.6.3
mkdir build
cd build
cmake .. 
make -j$(nproc)
sudo make install
```

After installation, the _Build_ directory can be removed.