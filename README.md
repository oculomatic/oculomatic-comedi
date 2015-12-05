# master
Eye Tracking for point gray cameras

Simple and fast eye tracking for head restrained applications.

Tracks pupil location and centroid size.

Outputs are x and y raw position over analog voltage outputs via COMEDI (NI DAQ board or similar required).

Performance metrics coming soon.

REQUIRES openCV > 3.0; TBB; BOOST; COMEDI..

Build openCV with CXXFLAGS="-ffast-math -march=native" CFLAGS=$CXXFLAGS cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON ..

cmake .

make

Thanks to Bijan Paeseran for the cameras
