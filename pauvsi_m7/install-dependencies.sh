# INSTALL FlyCapture2 DEPENDENCIES
sudo apt-get install libraw1394-11 libgtkmm-2.4-dev libglademm-2.4-dev libgtkglextmm-x11-1.2-dev libgtkglextmm-x11-1.2 libusb-1.0-0

#installs the flycapture library
roscd pauvsi_m7/FlyCapture/flycapture2.2
sudo sh install_flycapture.sh
cd

#Dependencies for ethzasl_ptam
sudo apt-get install freeglut3-dev liblapacke-dev liblapack3 libopenblas-base libopenblas-dev liblapack-dev

# install rosserial
sudo apt-get install ros-kinetic-rosserial
sudo apt-get install ros-kinetic-rosserial-arduino

