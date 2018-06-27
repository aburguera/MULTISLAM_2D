figure;
% Load externally detected loop closings
loopClosings=load('data/loopclosings.csv');
% Load odometric data of first session
load data/odoData;
% Do first session
subplot(1,2,1);
slamData1=mslam(odoData,loopClosings,[],1,size(odoData,2));
% Load odometric data of second session
load data/odoData2;
% Do second session
subplot(1,2,2);
slamData2=mslam(odoData,loopClosings,slamData1,1,size(odoData,2));

