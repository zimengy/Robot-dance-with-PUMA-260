%% PUMA Dance
%
% Starter code by Katherine J. Kuchenbecker
% MEAM 520 at the University of Pennsylvania


%% Clean up

% Clear all variables and functions.  You should do this before calling any PUMA functions.
clear all

% Move the cursor to the top of the command window so new text is easily seen.
home


%% Definitions

% Define team number.
teamnumber = 123;

% Define student names.
studentnames = 'Zimeng Yang and Bokang Wang';

% Load the dance file from disk.
load team123

% Pull the list of via point times out of the dance matrix for use below.
tvia = dance(:,1);

% Initialize the function that calculates angles.
team123_get_angles

% Define music filename (without team number).
musicfilename = 'East Ender G';


%% Music

% Load the piece of music for the robot to dance to.
[y,Fs] = audioread([num2str(teamnumber) ' ' musicfilename '.wav']);

% Calculate the duration of the music.
musicduration = (length(y)-1)/Fs;

% Calculate the duration of silence at the start of the dance.
silenceduration = abs(min(tvia));

% Create a time vector for the entire piece of music, for use in plotting.
t = ((min(tvia):(1/Fs):musicduration))';

% Pad the start of the music file with zeros for the silence.
y = [zeros(length(t)-length(y),2); y];


%% Choose duration

% Set the start and stop times of the segment we want to test.
% To play the entire dance, set tstart = t(1) and tstop = t(end).
tstart = t(1);
tstop = t(end);

% Select only the part of the music that we want to play right now, from
% tstart to tstop.
yplay = y(1+round(Fs*(tstart - t(1))):round(Fs*(tstop-t(1))),:);

% Put this snippet into an audio player so we can listen to it.
music = audioplayer(yplay,Fs);


%% Plot music

% Pull first audio channel and downsample it for easier display.
factordown = 30;
ydown = downsample(y(:,1),factordown);

% Downsample the time vector too.
tdown = downsample(t,factordown);

% Open figure and clear it.
figure(2)
clf

% Plot one of the sound channels to look at.
plot(tdown,ydown,'Color',[.2 .4 0.8]);
xlim([floor(t(1)) ceil(t(end))])

% Turn on hold to allow us to plot more things on this graph.
hold on

% Plot a vertical line at the time of each of the via points.
for i = 1:length(tvia)
    plot(tvia(i)*[1 1],ylim,'k--')
end

% Plot vertical lines at the start and stop times.
plot(tstart*[1 1],ylim,'k:')
plot(tstop*[1 1],ylim,'k:')

% Add a vertical line to show where we are in the music.
hline = plot(tstart*[1 1],ylim,'r-');

% Turn off hold.
hold off

% Set the title to show the team number and the name of the song.
title(['Team ' num2str(teamnumber) ': ' musicfilename])


%% Start robot

% Open figure 1 and clear it.
figure(1)
clf

% Initialize the PUMA simulation.
pumaStart()

% Set the view so we are looking from roughly where the camera will be.
view(80,20)

% The PUMA always starts with all joints at zero except joint 5, which
% starts at -pi/2 radians.  You can check this by calling pumaAngles.
thetahome = pumaAngles;

% Call pumaServo once to initialize timers.
pumaServo(thetahome);


%% Initialize dance

% Calculate the joint angles where the robot should start.
[thetastart, no_use] = team123_get_angles(tstart);

% Calculate time needed to get from home pose to starting pose moving at
% angular speed of 0.5 radians per second on the joint that has the
% farthest to go.
tprep = abs(max(thetastart - thetahome)) / .5;

% Start the built-in MATLAB timer.
tic

% Slowly servo the robot to its starting position, in case we're
% not starting at the beginning of the dance.
while(true)
    % Get the current time for preparation move.
    tnow = toc;
    
    % Check to see whether preparation move is done.
    if (tnow > tprep)
        
        % Servo the robot to the starting pose.
        pumaServo(thetastart)
        
        % Break out of the infinite while loop.
        break

    end
    
    % Calculate joint angles.
    thetanow = team123_linear_trajectory(tnow,0,tprep,thetahome,thetastart);

    % Servo the robot to this pose to prepare to dance.
    pumaServo(thetanow);
end

% Initialize history vectors for holding time and angles.  We preallocate
% these for speed, making them much larger than we will need.
thistory = zeros(10000,1);
thetahistory = zeros(10000,6);
velohistory = zeros(10000,6);

% Initialize our counter at zero.
i = 0;


%% Start music and timer

% Start the zero-padded music so we hear it.
play(music);

% Start the built-in MATLAB timer so we can keep track of where we are in
% the song.
tic


%% Dance

% Enter an infinite loop.
while(true)
    % Increment our counter.
    i = i+1;
    
    % Get the current time elapsed and add it to the time where we're
    % starting in the song. Store this value in the thistory vector.   
    thistory(i) = toc + tstart;
    % Check if we have passed the end of the performance.
    if (thistory(i) > tstop)
        
        % Break out of the infinite while loop.
        break

    end
    
    % Calculate the joint angles and joint velocities for the robot at this
    % point in time and store in our thetahistory matrix and velohistory matrix
    [thetahistory(i,:), velohistory(i,:)] = team123_get_angles(thistory(i));
    
    % Decide whether all joint angle limits are obeyed
    if thetahistory(i,1)<=-pi || thetahistory(i,1) >= 110/180*pi
        error('Joint 1 exceeds joint angle limits.');
    end
    if thetahistory(i,2)<=-75/180*pi || thetahistory(i,2) >= 240/180*pi
        error('Joint 2 exceeds joint angle limits.');
    end 
    if thetahistory(i,3)<=-235/180*pi || thetahistory(i,3) >= 60/180*pi
        error('Joint 3 exceeds joint angle limits.');
    end
    if thetahistory(i,4)<=-580/180*pi || thetahistory(i,4) >= 40/180*pi
        error('Joint 4 exceeds joint angle limits.');
    end
    if thetahistory(i,5)<=-120/180*pi || thetahistory(i,5) >= 110/180*pi
        error('Joint 5 exceeds joint angle limits.');
    end
    if thetahistory(i,6)<=-215/180*pi || thetahistory(i,6) >= 295/180*pi
        error('Joint 6 exceeds joint angle limits.');
    end
    
     
    % Servo the robot to these new joint angles.
    pumaServo(thetahistory(i,:));
    % Move the line on the music plot.
    set(hline,'xdata',thistory(i)*[1 1]);
end

% Stop the PUMA robot.
pumaStop
        
% Remove the unused ends of the history vector and matrix, which we
% preallocated for speed.
thistory(i:end) = [];
thetahistory(i:end,:) = [];
velohistory(i:end,:)=[];


%% Plot output

% Open figure 3 and clear it.
figure(3)
clf
subplot(2,1,1)
% Plot the joint angles and joint velocities versus time to make it easy to
% show how they changed during the dance.  You should use the variables
% thetahistory and thistory for this.
plot(thistory, thetahistory(:,1),'b');hold on;
plot(thistory, thetahistory(:,2),'r');hold on;
plot(thistory, thetahistory(:,3),'y');hold on;
plot(thistory, thetahistory(:,4),'k');hold on;
plot(thistory, thetahistory(:,5),'g');hold on;
plot(thistory, thetahistory(:,6),'m');hold on;
axis([thistory(1) thistory(end) 1.1*min(min(thetahistory)) 1.1*max(max(thetahistory))])
% Plot a vertical line at the time of each of the via points
for i = 1:length(tvia)
    plot(tvia(i)*[1 1],ylim,'--','color',[0.6,0.6,0.6])
end
title(['Team ' num2str(teamnumber) ': Joint Angles over Time'])


subplot(2,1,2)
plot(thistory, velohistory(:,1),'b');hold on;
plot(thistory, velohistory(:,2),'r');hold on;
plot(thistory, velohistory(:,3),'y');hold on;
plot(thistory, velohistory(:,4),'k');hold on;
plot(thistory, velohistory(:,5),'g');hold on;
plot(thistory, velohistory(:,6),'m');hold on;
axis([thistory(1) thistory(end) 1.1*min(min(velohistory)) 1.1*max(max(velohistory))])
% Plot a vertical line at the time of each of the via points
for i = 1:length(tvia)
    plot(tvia(i)*[1 1],ylim,'--','color',[0.6,0.6,0.6])
end
title(['Team ' num2str(teamnumber) ': Joint Velocities over Time'])
