%% Multi-lane Traffic Simulation
clear; close all; clc;

% Add path for function
addpath(genpath('C:/Users/monea/OneDrive/Documents/MATLAB/I-80-Emeryville-CA/vehicle-trajectory-data/0500pm-0515pm/kalman_filter/function'))

PltRoads=[];

InitVals;
InitCodes;
fm=1;
rng(11);
Zone=zeros(3,3);
CarOut=0;
CarIn=0;
CarMrg=0;
tic; %Tmrg=T+1; Cmrg=0;

Case=[0,80; -10,120];
RandX0=Case(1,1);
RandXR0=Case(1,2);

RandX=[100;100]+RandX0;
RandXR=160;

%% Video
outputVideo = VideoWriter('C:/Users/monea/OneDrive/Documents/MATLAB/I-80-Emeryville-CA/vehicle-trajectory-data/0500pm-0515pm/kalman_filter/data/MultiLaneSimulation.avi'); % Specify file name and format
outputVideo.FrameRate = 10; % Set frame rate (adjust based on your simulation speed)
open(outputVideo); % Open the video writer for writing frames

%% Congested
for I=1:5
    L1(end+1,1)=Car(20000+I,1,L1(end).X-70);
    L1(end).OgLnID = 1;  % Ensure original lane ID is set to 2 (blue)
    if(I<5)
        LR(end+1,1)=Car(30000+I,2,LR(end).X-100);
        LR(end).OgLnID = 2;  % Ensure original lane ID is set to 2 (blue)
    end
end

plt=[];
NextCarIDLane=[L1(end).ID+1; LR(end).ID+1];

hFig=figure;
set(hFig, 'position', [20,300,1500,120]);
ti=cputime;

CarInPer10s=0;

 %% Main
for i=1:181 % 2sec*60/0.5 + 1= 241 % 1:2401(20.01sec or 1200.5ms)

    if(mod(i,2)==1) %% Control how frequently you want to display vehicles.
        SimPlots3Lane3n;
    end

    % Capture the current frame and write it to the video
    frame = getframe(hFig); % Capture the figure as a frame
    writeVideo(outputVideo, frame); % Write the frame to the video

    if(mod(i,20)==0)
        [i*dt, CarIn-CarInPer10s]; % answer
        CarInPer10s=CarIn;
    end
    %% In OUT of Cars
    % if(L1(1).X>MPCrng)
    %     %%Store
    %     L1(1)=[]; 
    %     CarOut=CarOut+1;
    % end

    % add car to L1
    % if(L1(end).X>-60)
    %     L1(end+1,1)=Car(NextCarIDLane(1),2,L1(end).X-45);
    %     NextCarIDLane(1)=NextCarIDLane(1)+1;CarIn=CarIn+1;
    % end
    % 
    
    % add car to LR
    % if(LR(end).X>200)
    %     Agap=75;
    %     LR(end+1,1)=Car(NextCarIDLane(2),3,LR(end).X-Agap-round(0.08+rand/2,0)*100);
    %     NextCarIDLane(2)=NextCarIDLane(2)+1;
    % end

    %% Calculate acceleration of cars
    for I=1:length(L1)
        L1(I).Lpos=I;
        %save trajectory
        if(I>1) 
            L1(I).Ac=L1(I).IDM(L1(I),L1(I-1));
        else 
            L1(I).Ac=0.5*(L1(I).Vd-L1(I).V);
        end
    end

    for I=2:length(LR)
        LR(I).Lpos=I;
        %save trajectory
        LR(I).Ac=LR(I).IDM(LR(I),LR(I-1));
    end

    %% Update States of All Cars
    for I=1:length(L1)
        if( L1(I).Y>2.01) 
            L1(I).Y=2+ 0.9*(L1(I).Y-2); 
        end

        if( L1(I).Y<1.99)
            L1(I).Y=2- 0.9*(2-L1(I).Y);
        end
        Xp=L1(I).X;
        L1(I).X=L1(I).fdx(L1(I));
        L1(I).V=L1(I).fdv(L1(I));

        % Store normal state for L1 cars
        L1(I).storeNormalState(L1(I));  % Pass the current simulation time
    end

    for I=2:length(LR)
        Xp=LR(I).X;
        LR(I).X=LR(I).fdx(LR(I));
        LR(I).V=LR(I).fdv(LR(I));
        % Store normal state for LR cars
        LR(I).storeNormalState(LR(I));  % Pass the current simulation time
    end

    %% MOBIL Lane Change
    CnL = length(LR);  % Count of cars in LR (merging lane)
    Ylc = 0;  % Lane change flag
    minGapAhead = 20;  % Minimum gap required in front (meters)
    minGapBehind = 20;  % Minimum gap required behind (meters)

    for I = 2:CnL  % Check for the first few cars in LR min(CnL - 2, 10)
        if(I > CnL || LR(I).X < 1000)  % Stop if car is too far or out of range
            break; 
        end
        
        % Case 1: L1 is empty
        if isempty(L1)
            LR(I).LnID = 1;  % Update lane ID
            LR(I).Y=LR(I).Y + 0.6; % smooth merging animation
            L1(end + 1) = LR(I);  % Add car to L1
            LR(I) = [];  % Remove car from LR
            CarMrg = CarMrg + 1;  % Increment merge count
            Ylc = 2;  % Mark lane change as completed
            break;
        end

        % Case 2: Single car in L1
        if length(L1) == 1
            % Compute the gap between the single car in L1 and the car in LR
            if LR(I).X < L1(1).X - 50  % Ensure safe distance to merge
                LR(I).LnID = 1;  % Update lane ID
                LR(I).Y=LR(I).Y + 0.6; % smooth merging animation
                L1(end + 1) = LR(I);  % Add car to L1
                LR(I) = [];  % Remove car from LR
                CarMrg = CarMrg + 1;
                Ylc = 2;  % Mark lane change as completed
                break;
            else
                continue;  % Skip if the car in LR is too close to L1 car
            end
        end

        % Check and decrement lane change flag
        if Ylc > 0
            Ylc = max(Ylc - 1, 0);  % Decrease lane change flag
            continue;  % Skip this iteration until flag resets
        end

        % Skip the ghost car with ID 20000
        if LR(I).ID == 20000
            continue;
        end
    
        % Ensure only the first unmerged car in ID order merges
        earlierCars = [LR(1:I-1).ID];  % IDs of earlier cars in LR
        earlierCars = earlierCars(earlierCars ~= 20000);  % Ignore the ghost car
        if ~isempty(earlierCars) && any(ismember(earlierCars, [LR(:).ID]))  % If any earlier car hasn't merged
            continue;  % Wait until earlier cars merge
        end

        % Case 3: Multiple cars in L1
        TL1 = [L1(1:end).X]' - LR(I).X;  % Gaps between LR car and L1 cars
        Icxp = sum(TL1 > 0);  % Find the first car in L1 ahead of LR
    
        % Handle invalid or out-of-bounds Icxp
        if Icxp == 0 || Icxp > length(L1)
            Icxp = length(L1);  % Default to the last car in L1
        end

        gapAhead = inf;  % Default to large value if no car ahead
        gapBehind = inf; % Default to large value if no car behind
        
        if Icxp > 0 && Icxp <= length(L1)  % Check gap ahead
            gapAhead = L1(Icxp).X - LR(I).X;
        end
        if Icxp > 1  % Check gap behind
            gapBehind = L1(Icxp - 1).X - LR(I).X;  % Distance to the car behind in L1
        end
    
        % Skip merging if gaps are insufficient
        disp(['Car ', num2str(LR(I).ID), ' gap_ahead: ', num2str(gapAhead), ', gap_behind: ', num2str(gapBehind)]);
        disp(['Current Merge Conditions: gap_ahead > ', num2str(minGapAhead), ' && gap_behind > ', num2str(minGapBehind)]);
        if gapAhead > minGapAhead && gapBehind > minGapBehind
            disp(['Car ', num2str(LR(I).ID), ' can merge']);
        else
            disp(['Car ', num2str(LR(I).ID), ' cannot merge: insufficient gaps']);
            continue;
        end
    
        % Acceleration-based merging logic
        ah = IDM4LC(LR(I), LR(I - 1));  % Acceleration of LR car with respect to previous car in LR
        ao = 0;  % Default acceleration for no car ahead in LR
        if I < CnL
            ao = IDM4LC(LR(I + 1), LR(I));  % Acceleration of next car in LR
        end
    
        an = 0;  % Default acceleration for no car ahead in L1
        if Icxp <= length(L1) - 1
            an = IDM4LC(L1(Icxp + 1), L1(Icxp));  % Acceleration of next car in L1
        end
    
        ah1 = IDM4LC(LR(I), L1(Icxp));  % Acceleration of LR car if it moves into L1
        
        if Icxp + 1 <= length(L1)
            an1 = IDM4LC(L1(Icxp + 1), LR(I));  % Acceleration of next L1 car if LR merges
        else
            an1 = 0;  % Default or fallback value
        end
    
        % Merge condition
        if ah < ah1 + 2.0 && an1 > -6.0
            LR(I).LnID = 1;  % Update lane ID
            LR(I).Y=LR(I).Y + 0.6; % smooth merging animation
            L1(Icxp + 2:end + 1) = L1(Icxp + 1:end);  % Shift cars in L1
            L1(Icxp + 1) = LR(I);  % Add car to L1
            disp(['Car ', num2str(LR(I).ID), ' merged']);
            LR(I) = [];  % Remove car from LR
            CarMrg = CarMrg + 1;  % Increment merge count
            Ylc = 2;  % Mark lane change as completed
            break;
        end
    end
end

close(outputVideo); % Finalize and save the video file
disp('Video saved as MultiLaneSimulation.avi');

