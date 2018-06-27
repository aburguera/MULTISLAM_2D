% Name        : slamData=mslam(odoData,loopClosings,slamPrev,firstFrame,lastFrame)
% Description : 2D Multisession Visual SLAM
% Input       : odoData    - Odometric data as provided by
%                            compute_odometry. See https://github.com/aburguera/VISUAL_ODOMETRY_2D
%                            for more information.
%               loopClosings - Internal (within the current odoData) and
%                            external (involving slamPrev data) loop
%                            closings obtained by external means. Format
%                            is: first column and second column are the
%                            image numbers (imNum) of the images that close
%                            the loop. Third column states the quality of
%                            the loop (the higher the better), though is
%                            not used in this implementation.
%               slamPrev   - SLAM data structure of a previous SLAM
%                            session. Format is the same as the output of
%                            this same function.
%               firstFrame - Index within odoData of the first odometric
%                            estimate to use to perform SLAM.
%               lastFrame  - Index within odoData of the last odometric
%                            estimate to use to perform SLAM.
% Output      : slamData   - Structure with the following fields:
%                          - X: State vector. The items X(i*3-2:i*3) denote
%                            the motion in x, y and orientation from frame
%                            i-1 to frame i. X(1:3,1) are zero. Note that
%                            X(i*3-2:i*3,1) is directly related to
%                            odoData(i) until the two sessions are joined.
%                          - P: Covariance of X.
%                          - F: Image numbers so that F(i) contains the
%                            image number associated to X(i*3-2:i*3,1).
%                          - theFeatures: Array of structures. The i-th
%                            structure has two fields (f and d) holding the
%                            SIFT features and descriptors associated to
%                            X(i*3-2:i*3,1).
%                          - slamPrev: SLAM data in this same format of the
%                            previous SLAM session. This field is cleaned
%                            when the two sessions are joined.
%                          - theLinks: Image numbers of the positions that
%                            join the sessions. Used to plot.
%                          - separatedMaps: If <>0 means that the two
%                            sessions have still not been joined.
%                          - externalLoops: Image numbers of the detected
%                            external loops. Used only for logging/plotting
%                            purposes.
%                          - internalLoops: Same as before, but for
%                            internal loops.
% Author      : Antoni Burguera Burguera
%               antoni.burguera@uib.es
% Note        : Please, refer to the README.TXT file for information about
%               how to properly cite us if you use this software.
function slamData=mslam(odoData,loopClosings,slamPrev,firstFrame,lastFrame)
    % Initialize system
    globalData=init(odoData,loopClosings,firstFrame,lastFrame);
    slamData=slam_init(slamPrev);
    externalLoop=external_loop_clean();

    % Loop for each frame
    while more_frames(globalData)
        % Add odometric estimate to the state vector
        slamData=slam_augment_state(slamData,globalData);
        % Search internal and external loops
        [internalMeasurements,externalMeasurements]=search_loops(slamData,globalData);
        % Accumulate external loops
        [externalLoop,joinMaps]=external_loop_update(slamData,globalData,externalLoop,externalMeasurements);
        % If enough external loops and previous SLAM still separated
        if joinMaps
            % Join maps
            slamData=slam_join(slamData,globalData,externalLoop);
        end;
        % Apply internal loops
        slamData=slam_update(slamData,globalData,internalMeasurements);
        % Plot SLAM
        cla;
        draw_mslam(slamData);
        drawnow;
        % Go to next frame
        globalData=next_frame(globalData);
    end;
return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% PARAMETER INITIALIZATION %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define global parameters
function globalData=init(odoData,loopClosings,firstFrame,lastFrame)
    TwoSigmaPos=2;
    TwoSigmaAngle=3*pi/180;
    % Odometry covariance
    globalData.Podo=[(TwoSigmaPos/2)^2,0,0;0,(TwoSigmaPos/2)^2,0;0,0,(TwoSigmaAngle/2)^2];
    % Loop closing covariance
    globalData.Ploop=globalData.Podo;
    % Maximum distance to search loops
    globalData.loopMaxDistance=500;
    % Required loop closings to join maps
    globalData.loopsToJoin=25;
    % Pre-computed loop closings
    globalData.loopClosings=loopClosings;
    % Odometric information
    globalData.odoData=odoData;
    % First and last frames to use
    globalData.firstFrame=firstFrame;
    globalData.lastFrame=lastFrame;
    % Current odometry frame
    globalData.curFrame=firstFrame;
    % Number of iterations for RANSAC motion estimation
    globalData.ransacIter=100;
return;

%%%%%%%%%%%%
%%% SLAM %%%
%%%%%%%%%%%%

% Initialize SLAM data
function slamData=slam_init(slamPrev)
    % State vector. X(i*3-2:i*3) contains the roto-translation from frame
    % i-1 to frame i. X(1:3) is (0,0,0).
    slamData.X=[];
    % State covariance
    slamData.P=[];
    % Image number corresponding to frame i.
    slamData.F=[];
    % SIFT features. theFeatures(i).f and theFeatures(i).d are the
    % coordinates and descriptors of the SIFT features of the image in
    % frame i.
    slamData.theFeatures=[];
    % Previous SLAM session
    slamData.slamPrev=slamPrev;
    % Indexes to the links between maps. Initialized to one to ease plot
    % when no links available.
    slamData.theLinks=1;
    % If 1, this session has still not been joined with the previous one.
    slamData.separatedMaps=1;
    % External loops, just for log and plot
    slamData.externalLoops=[];
    % Internal loops, just for log and plot
    slamData.internalLoops=[];
return;

% Augment SLAM state with current odometric estimate.
function slamData=slam_augment_state(slamData,globalData)
    % Include current odometric estimate into state vector
    slamData.X=[slamData.X;globalData.odoData(globalData.curFrame).X];
    % Augment covariance matrix
    slamData.P(end+1:end+3,end+1:end+3)=globalData.Podo;
    % Store image number
    slamData.F=[slamData.F globalData.odoData(globalData.curFrame).imNum];
    % Store features and descriptors
    slamData.theFeatures(end+1).f=globalData.odoData(globalData.curFrame).f;
    slamData.theFeatures(end).d=globalData.odoData(globalData.curFrame).d;
return;

% Optimize trajectory (IEKF) according to internal loop closures.
function slamData=slam_update(slamData,globalData,theMeasurements)
    % Store internal loops (logging purposes)
    slamData.internalLoops=[slamData.internalLoops [theMeasurements.F;theMeasurements.Fe]];
    frameIndexes=imnum2idx(theMeasurements.F,slamData.F);
    % Compute the predicted measurements (i.e. h in IEKF)
    predictedMeasurements=zeros(3,size(frameIndexes,2));
    for i=1:size(frameIndexes,2)
        X=zeros(3,1);
        for j=frameIndexes(:,i):(size(slamData.X,1)/3)-1,
            [X,~]=compose_references(X,slamData.X((j+1)*3-2:(j+1)*3,1),[],[]);
        end;
        predictedMeasurements(:,i)=X;
    end;
    % Iterate (IEKF)
    for j=1:10,  
        H=[];  
        theInnovation=zeros(size(theMeasurements.Z,2)*3,1);
        Rsm=zeros(size(theMeasurements.Z,2)*3);
        Xtmp=slamData.X;
        Ptmp=slamData.P;
        % For each measurement
        for i=1:size(theMeasurements.Z,2),
            % Compute partial Jacobian matrix
            [Htmp]=compute_observation_jacobian(Xtmp, predictedMeasurements(:,i), frameIndexes(:,i)+1);
            % Accumulate it
            H=[H;Htmp];
            % Prepare data
            theMeasurement=theMeasurements.Z(:,i);
            thePrediction=predictedMeasurements(:,i);
            theDifference=theMeasurement-thePrediction;
            theDifference(3,1)=normalize(theDifference(3,1));
            i1=i*3;
            i0=i1-2;
            theInnovation(i0:i1,1)=theDifference;
            Rsm(i0:i1,i0:i1)=globalData.Ploop;
        end;
        % IEKF update
        if (size(theMeasurements.Z,2)>0),
            tmp=slamData.P*H';
            Ptmp=slamData.P-tmp*(inv(H*tmp+Rsm))*H*slamData.P;
            Xtmp=Xtmp+Ptmp*H'*(inv(Rsm))*(theInnovation)-Ptmp/slamData.P*(Xtmp-slamData.X);              
        end;
    end;
    slamData.X=Xtmp;
    slamData.P=Ptmp;          
return;

% Join the two sessions
function slamData=slam_join(slamData,globalData,theMeasurements)
    % Join and augment external loops (logging only(
    slamData.externalLoops=[slamData.slamPrev.externalLoops slamData.externalLoops [theMeasurements.F;theMeasurements.Fe]];
    % Optimization to compute the link
    [theLink,Plink]=compute_link(slamData,globalData,theMeasurements);
    % Join state and covariance through the link
    newX=[slamData.slamPrev.X;theLink;slamData.X];
    newP=slamData.slamPrev.P;
    newP(end+1:end+3,end+1:end+3)=Plink;
    tmp=size(slamData.P,1);
    newP(end+1:end+tmp,end+1:end+tmp)=slamData.P;
    % Join frame ids. Place a -1 to the link, since it has no image.
    newF=[slamData.slamPrev.F -1 slamData.F];
    % Join features
    newFeatures=slamData.slamPrev.theFeatures;
    newFeatures(end+1).f=[];
    newFeatures(end+1:end+size(slamData.theFeatures,2))=slamData.theFeatures;
    % Store data
    slamData.X=newX;
    slamData.P=newP;
    slamData.F=newF;
    slamData.theFeatures=newFeatures;
    % Join links
    slamData.theLinks=[slamData.slamPrev.theLinks size(slamData.slamPrev.X,1)/3+1];
    % Join internal loops
    slamData.internalLoops=[slamData.slamPrev.internalLoops slamData.internalLoops];
    % State maps are no longer sepparated.
    slamData.separatedMaps=0;
    % Clean previous SLAM data
    slamData.slamPrev=[];
return;

%%%%%%%%%%%%%%%%%%%
%%% LOOP SEARCH %%%
%%%%%%%%%%%%%%%%%%%

% Search internal and external loops
function [internalMeasurements,externalMeasurements]=search_loops(slamData,globalData)
    % Initialize parameters and storage
    numPoses=size(slamData.X,1)/3;
    X=slamData.X(end-2:end,1);
    internalMeasurements.F=[];
    internalMeasurements.Fe=[];
    internalMeasurements.Z=[];
    externalMeasurements.F=[];
    externalMeasurements.Fe=[];
    externalMeasurements.Z=[];

    % Search internal loops with RANSAC
    for i=numPoses-2:-1:1
        curSLAM=slamData.X((i+1)*3-2:(i+1)*3,1);
        [X,~]=compose_references(curSLAM,X,[],[]);
        if sqrt(X(1:2)'*X(1:2))<globalData.loopMaxDistance && ~isempty(slamData.theFeatures(i).d)
            [matches,~]=vl_ubcmatch(slamData.theFeatures(i).d,slamData.theFeatures(numPoses).d);
            if size(matches,2)>10
                [Z,fail]=ransac_estimate_motion(slamData.theFeatures(i).f(1:2,matches(1,:)),slamData.theFeatures(numPoses).f(1:2,matches(2,:)),globalData.ransacIter,5,10,.75);
                if ~fail
                    internalMeasurements.F=[internalMeasurements.F slamData.F(i)];
                    internalMeasurements.Fe=[internalMeasurements.Fe slamData.F(end)];
                    internalMeasurements.Z=[internalMeasurements.Z Z];
                end;
            end;
        end;
    end;
    
    % Search loops with externally provided loop closings
    curImNum=slamData.F(numPoses);
    candidateLoops=globalData.loopClosings(globalData.loopClosings(:,1)==curImNum,2);
    for j=1:size(candidateLoops,1)
        curLoop=candidateLoops(j);
        % Search internal loops
        if ismember(curLoop,slamData.F)
            if ~ismember(curLoop,internalMeasurements.F)
                internalMeasurements.F=[internalMeasurements.F curLoop];
                internalMeasurements.Fe=[internalMeasurements.Fe slamData.F(end)];
                i=find(slamData.F==curLoop);
                [matches,~]=vl_ubcmatch(slamData.theFeatures(i).d,slamData.theFeatures(numPoses).d);
                [Z,~]=ransac_estimate_motion(slamData.theFeatures(i).f(1:2,matches(1,:)),slamData.theFeatures(numPoses).f(1:2,matches(2,:)),globalData.ransacIter,5,10,.75);
                internalMeasurements.Z=[internalMeasurements.Z Z];
            end;
        % Search external loops
        elseif slamData.separatedMaps && ismember(curLoop,slamData.slamPrev.F)
            externalMeasurements.F=[externalMeasurements.F curLoop];
            externalMeasurements.Fe=[externalMeasurements.Fe slamData.F(end)];
            i=find(slamData.slamPrev.F==curLoop);
            [matches,~]=vl_ubcmatch(slamData.slamPrev.theFeatures(i).d,slamData.theFeatures(numPoses).d);
            [Z,~]=ransac_estimate_motion(slamData.slamPrev.theFeatures(i).f(1:2,matches(1,:)),slamData.theFeatures(numPoses).f(1:2,matches(2,:)),globalData.ransacIter,5,10,.75);
            externalMeasurements.Z=[externalMeasurements.Z Z];
        end;
    end;
return;

%%%%%%%%%%%%
%%% PLOT %%%
%%%%%%%%%%%%

% Draw trajectories and loops.
% Note that uncertainty ellipse is not correct since the robot covariance
% is not properly marginalized.
function draw_mslam(slamData)
    % Compute global poses
    X=zeros(3,1);
    P=zeros(3);
    Xh=zeros(3,size(slamData.X,1)/3);
    for i=1:size(slamData.X,1)/3
        [X,P]=compose_references(X,slamData.X(i*3-2:i*3,1),P,slamData.P(i*3-2:i*3,i*3-2:i*3));
        Xh(:,i)=X;
    end;
    
    % Draw internal loops
    if ~isempty(slamData.internalLoops)
        L=[imnum2idx(slamData.internalLoops(1,:),slamData.F);imnum2idx(slamData.internalLoops(2,:),slamData.F)];
        theX=[Xh(1,L(1,:));Xh(1,L(2,:))];
        theY=[Xh(2,L(1,:));Xh(2,L(2,:))];
        plot(theX,theY,'k');
        hold on;
    end;

    % Draw external loops
    if ~isempty(slamData.externalLoops)
        L=[imnum2idx(slamData.externalLoops(1,:),slamData.F);imnum2idx(slamData.externalLoops(2,:),slamData.F)];
        theX=[Xh(1,L(1,:));Xh(1,L(2,:))];
        theY=[Xh(2,L(1,:));Xh(2,L(2,:))];
        plot(theX,theY,'r','LineWidth',2);
        hold on;
        plot(theX(1,:),theY(1,:),'ro');hold on;
        plot(theX(2,:),theY(2,:),'ro');hold on;
    end;
    
    % Draw each trajectory
    for i=2:size(slamData.theLinks,2)
        plot(Xh(1,slamData.theLinks(i-1):slamData.theLinks(i)-1),Xh(2,slamData.theLinks(i-1):slamData.theLinks(i)-1),'LineWidth',2);
        hold on;
    end;
    plot(Xh(1,slamData.theLinks(end):end),Xh(2,slamData.theLinks(end):end),'LineWidth',2);
    
    % Plot orientations at some points
    for i=1:10:size(Xh,2)
        draw_vehicle(Xh(:,i),50,'b');
        hold on;
    end;
    % Draw uncertainty ellipse (simple marginalization)
    draw_ellipse(Xh(:,end),P,'r');
    hold on;
    % Draw current orientation
    draw_vehicle(Xh(:,end),100,'r');
    axis equal;
return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% OPTIMIZATION RELATED FUNCTIONS %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Optimize the transformation from the end of previous map to the beginning
% of the current one by means of IEKF.
function [theLink,Plink]=compute_link(slamData,globalData,theMeasurements)
    % Compute initial estimate
    Z=theMeasurements.Z(:,1);
    [prePos,curPos]=compute_precur(slamData,theMeasurements,1);
    [X0,~]=invert_reference(prePos,[]);
    [X1,~]=invert_reference(curPos,[]);
    [theLink,~]=compose_references(X0,Z,[],[]);
    [theLink,~]=compose_references(theLink,X1,[],[]);
    Plink=globalData.Ploop;
    
    % Update
    nMeasurements=size(theMeasurements.Z,2);
    Z=reshape(theMeasurements.Z,nMeasurements*3,1);
    for j=1:100
        h=[];
        H=[];
        Rsm=[];
        for i=1:nMeasurements
            [Xie,Xsj]=compute_precur(slamData,theMeasurements,i);
            c1=cos(Xie(3));
            s1=sin(Xie(3));
            c2=cos(Xie(3)+theLink(3));
            s2=sin(Xie(3)+theLink(3));
            hij=[Xie(1)+theLink(1)*c1-theLink(2)*s1+Xsj(1)*c2-Xsj(2)*s2;
                 Xie(2)+theLink(1)*s1+theLink(2)*c1+Xsj(1)*s2+Xsj(2)*c2;
                 Xie(3)+theLink(3)+Xsj(3)];
            Hij=[c1,-s1,-Xsj(2)*c2-Xsj(1)*s2;
                 s1,c1,Xsj(1)*c2-Xsj(2)*s2;
                 0,0,1];
            h=[h;hij];
            H=[H;Hij];
            Rsm(end+1:end+3,end+1:end+3)=globalData.Ploop;
        end;

        theInnovation=Z-h;
        S=Rsm+H*Plink*H';
        K=Plink*H'*inv(S);
        theLink=theLink+K*theInnovation;
        tmp=eye(3)-K*H;
        Plink=tmp*Plink*tmp'+K*Rsm*K';
    end;
return;

% Helper function for SLAM update. Computes the Jacobian matrix of the
% observation function.
function H=compute_observation_jacobian(Xslam,hk,startIndex)
  % Build the first part of the Jacobian, which is composed of zeros
  H=zeros(3,3*(startIndex-1));
  % The rest of items
  glk=zeros(3,1);
  % Initialize c=cos(0), s=sin(0)
  c=1;
  s=0;
  for i=startIndex:size(Xslam,1)/3,
    % Compute glk
    Xs=Xslam(i*3-2:i*3,1);
    glk=[glk(1)+Xs(1)*c-Xs(2)*s;glk(2)+Xs(1)*s+Xs(2)*c;glk(3)+Xs(3)];
    % Precompute some parameters
    c=cos(glk(3));
    s=sin(glk(3));
    p1=[-glk(1)*c-glk(2)*s+hk(1)*c+hk(2)*s;glk(1)*s-glk(2)*c-hk(1)*s+hk(2)*c];
    p2=glk(3)-Xslam(i*3,1);
    sp=sin(p2);
    cp=cos(p2);
    % Store the partial Jacobian into the output Jacobian matric
    H=[H [1,0,-p1(1)*s-p1(2)*c;0,1,p1(1)*c-p1(2)*s;0,0,1]*[cp,-sp,0;sp,cp,0;0,0,1]];    
  end;
return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% HELPER/AUXILIARY CODE %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Delete external loop storage
function externalLoop=external_loop_clean()
    externalLoop.F=[];
    externalLoop.Z=[];
    externalLoop.Fe=[];   
return;

% Accumulate external loops and decide if maps can be joined
function [externalLoop,joinMaps]=external_loop_update(slamData,globalData,externalLoop,externalMeasurements)
    % Accumulate external loops
    if slamData.separatedMaps && size(externalMeasurements.F,2)>0
        externalLoop.F=[externalLoop.F externalMeasurements.F];
        externalLoop.Z=[externalLoop.Z externalMeasurements.Z];
        externalLoop.Fe=[externalLoop.Fe externalMeasurements.Fe];
    end;
    % State that maps can be joined if still sepparated and enough external
    % loops.
    joinMaps=slamData.separatedMaps && size(externalLoop.F,2)>=globalData.loopsToJoin;
return;

% Compute transformations from a given loop closure to the end of the
% previous map and from the beginning of the current map to the loop
% closure.
function [prePos,curPos]=compute_precur(slamData,theMeasurements,fIndex)
    F=imnum2idx(theMeasurements.F(fIndex),slamData.slamPrev.F);
    Fe=imnum2idx(theMeasurements.Fe(fIndex),slamData.F);
    curPos=compose_trajectory(slamData.X,1,Fe);
    prePos=compose_trajectory(slamData.slamPrev.X,F+1,size(slamData.slamPrev.X,1)/3);
return;

% Composes the tranaformations within a given interval of a state vector.
function X=compose_trajectory(Xslam,iStart,iEnd)
    X=zeros(3,1);
    for i=iStart:iEnd
        [X,~]=compose_references(X,Xslam(i*3-2:i*3,1),[],[]);
    end;
return;

% Converts from image number to index
function theIndexes=imnum2idx(imNum,Fslam)
    if isempty(imNum)
        theIndexes=[];
        return;
    end;
    theIndexes=[];
    for i=1:size(imNum,2)
        theIndexes=[theIndexes find(Fslam==imNum(i))];
    end;
    
return;

% Helper to check if last odometric data item has been reached.
function areThere=more_frames(globalData)
    areThere=globalData.curFrame<=globalData.lastFrame;
return;

% Move to next odometric data item
function globalData=next_frame(globalData)
    globalData.curFrame=globalData.curFrame+1;
return;

% Draw a triangle pointing towards the current X axis.
function draw_vehicle (X,robotSize,theColor)
  vertices = [1.5 -1 -1 1.5
              0    1 -1  0 ]*robotSize/2;
  vertices = compose_point(X(1:3), vertices);
  plot(vertices(1,:), vertices(2,:), theColor);hold on;
  plot(X(1), X(2), 'r.');
return;

% Draw 2 sigma ellipse
function draw_ellipse(X,P,theColor)
    tita=linspace(0,2*pi,20);
    theCircle=[cos(tita);sin(tita)];
    [V,D]=eig(full(P(1:2,1:2)));
    theAxes=sqrt(9.2103*diag(D));
    tmp=(V*diag(theAxes))*theCircle;
    hp=line(tmp(1,:)+X(1),tmp(2,:)+X(2));
    set(hp,'Color',theColor);
    set(hp,'LineWidth',1.5);
return;











