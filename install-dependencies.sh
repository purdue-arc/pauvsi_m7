#Dependencies for ethzasl_ptam
sudo apt-get install freeglut3-dev liblapacke-dev liblapack3 libopenblas-base libopenblas-dev liblapack-dev

# INSTALL TooN
cd ~/
git clone https://github.com/edrosten/TooN
cd TooN
./configure && make && sudo make install
cd ..


# Installing gvars
git clone https://github.com/edrosten/gvars
cd gvars
./configure && make && sudo make install


# Installing libcvd
cd ~/
git clone https://github.com/edrosten/libcvd
cd libcvd
./configure && make && sudo make install
make test

#installing ethzasl_ptam
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/ethzasl_ptam
cd ..

catkin_make
