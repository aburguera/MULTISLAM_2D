==============
=== CREDIT ===
==============

2D Multisession Visual SLAM

Author : Antoni Burguera
e-mail : antoni.burguera@uib.es

A paper describing the theory behind this software has been submitted to a peer-reviewer journal. The proper reference will be added to this file after acceptance.

If you use this software, please cite that reference. If the reference is not available in this README, please contact us.

====================
=== REQUIREMENTS ===
====================

This program requires VLFeat to be properly installed. Please, visit:

http://www.vlfeat.org/

for instructions about how to download and properly install it.

=========================
=== BUILD THE PACKAGE ===
=========================

Download and unpack the package, if necessary, wherever you want. Open Matlab and navigate to the package source directory. Type "ls" or "dir" and check that several .m files and one .c file are there. One of the .m files is "install.m".

The package is pure Matlab code except for one C file with MEX interface to Matlab. To build them, just execute the file "install.m":

>> install

Some warnings related to GCC version may appear, but no errors should occur. You should see "MEX completed successfully" appear.

In case of errors, check your Matlab MEX configuration by typing "help mex" and following the instructions.

If the problem persists, a pure Matlab implementation of the MEX file is provided. Just search calls to least_squares_cartesian_mex within all provided Matlab sources and change them to least_squares_cartesian.

====================
=== RUN THE CODE ===
====================

Within the source directory there is a "main.m" file. Execute it to see an example:

>> main

This example loads data from within the data directory. The required data is odoData.mat and odoData2.mat, which hold preprocessed visual odometry as provided by https://github.com/aburguera/VISUAL_ODOMETRY_2D

It also loads loopclosings.csv. This CSV file must have three columns. The first and the second column hold the image numbers that close a loop. These numbers are matched against the imNum field of odoData. The third column is supposed to be a quality measure for each loop, though it is unused by this implementation.

Even though main.m depends on some specific data, it shows the main usage. Roughly speaking, run this code to perform the first session:

>> slamData1=mslam(odoData,loopClosings,[],1,size(odoData,2));

This is equivalent of trajectory-based SLAM. For this first session, loopClosings is not mandatory. So, you can also do:

>> slamData1=mslam(odoData1,[],[],1,size(odoData,2));

The last two parameters define the start and end index to use within odoData.

To run the second session, you need the output of the first one. A second session can be executed like this

>> slamData2=mslam(odoData2,loopClosings,slamData1,1,size(odoData,2));

In this case, slamData1 is used to pass the previous map. Also, loopClosings is mandatory and must contain loops between the two maps. Otherwise, the maps won't be joined.

Navigate through the source code and read the comments and file headers to understand how the software works.

==================
=== PARAMETERS ===
==================

All the parameters are defined by the function

globalData=init(odoData,loopClosings,firstFrame,lastFrame)

Within this funcion, there are some parameters that you may need to modify:

* TwoSigmaPos and TwoSigmaAngle: The 2 sigma bounds for X, Y and Theta odometric estimates.
* loopsToJoin: How many external loops are accumulated before joining the maps.
* loopMaxDistance: The radius of the circle where loops are searched.

=======================
=== TROUBLESHOOTING ===
=======================

This software package has been tested using Matlab 8.5.0.197613 (R2015a) running on Ubuntu 16.04LTS with GCC 5.4.0. Other configurations are untested.