ARMarker

Installation:
1. Compile the ARToolkitt first
2. Change the path in the CMakerList.txt to your ARToolkit
4. OpenCV must be installed
5. make a build folder
   $ mkdir build 
6. run cmake in your build folder
   $ cd build
   $ cmake ..
7. compile armaker
   $ make
8. test it
   $ cd ..
   $ ./bin/armarker_sample ./examples/Data/1.pat
or
   $ ./bin/armarker_sample ./examples/Data/1.pat text.jpg
