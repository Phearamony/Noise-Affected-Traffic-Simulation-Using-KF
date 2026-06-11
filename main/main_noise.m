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

%% Ouput
output_directory = "C:\Users\monea\OneDrive\Documents\MATLAB\I-80-Emeryville-CA\vehicle-trajectory-data\0500pm-0515pm\kalman_filter\data";

%% Video
outputVideo = VideoWriter('C:/Users/monea/OneDrive/Documents/MATLAB/I-80-Emeryville-CA/vehicle-trajectory-data/0500pm-0515pm/kalman_filter/data/video/MultiLaneSimulation_Real_Noise1v6.avi'); % Specify file name and format
outputVideo.FrameRate = 10; % Set frame rate (adjust based on your simulation speed)
open(outputVideo); % Open the video writer for writing frames

%% Congested
for I=1:5
    L1(end+1,1)=Car(20000+I,1,L1(end).X-70);
    L1(end).OgLnID = 1;  % Ensure original lane ID is set to 2 (blue)
    % if(I<4)
    %     LR(end+1,1)=Car(30000+I,2,LR(end).X-100);
    %     LR(end).OgLnID = 2;  % Ensure original lane ID is set to 2 (blue)
    % end
end

plt=[];
NextCarIDLane=[L1(end).ID+1; LR(end).ID+1];

hFig=figure;
set(hFig, 'position', [20,300,1500,120]);
ti=cputime;

CarInPer10s=0;

% Initialize storage for total timing
num_iterations = 0;
total_time = 0;

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
            if(L1(I).OgLnID == 2 && L1(I-1).OgLnID == 1) % React to noise
                L1(I).Ac = L1(I).IDMN(L1(I), L1(I-1));
            else
                L1(I).Ac = L1(I).IDM(L1(I),L1(I-1));
            end
        else 
            L1(I).Ac = 0.5*(L1(I).Vd-L1(I).V);
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

    % Generate noise for cars originally in L1
    for I = 1:length(L1)
        if L1(I).OgLnID == 1
            L1(I).generateNoise(L1(I)); % Add noise to this car
        end
    end

    % 1. Normal LR Communicates with L1
    for J = 2:length(LR)
        if LR(J).OgLnID == 2
            for I = 1:length(L1)
                if L1(I).OgLnID == 1 % Communicate only with cars originally in L1
                    LR(J).updateV2VState(LR(J), L1(I)); % Update V2V state
                    LR(J).strV2VState(LR(J), L1(I));   % Store V2V data
                    if mod(i,30) == 0 || i == 181 % do it every 0.25 mn = 15 seconds
                        num_iterations = num_iterations + 1; % Count the iterations
                        
                        % Measure execution time for LR block
                        tic;

                        LR(J).v2vmodel(LR(J), L1(I)); % modelling
                        LR(J).KF(LR(J), L1(I)); % KF
                        LR(J).cred_v2v(LR(J), L1(I)); % cred
                        LR(J).reset_str(LR(J), L1(I)) % reset storage

                        total_time = total_time + toc; % Accumulate total time
                    end
                    LR(J).store_cred(LR(J), L1(I)); % store cred
                end
            end
        end
    end
    
    % 2. Merged LR (now in L1) Communicates with L1
    for J = 1:length(L1)
        if L1(J).OgLnID == 2 % Cars originally in LR but now in L1
            for I = 1:length(L1)
                if L1(I).OgLnID == 1 % Communicate only with cars originally in L1
                    L1(J).updateV2VState(L1(J), L1(I)); % Update V2V state
                    L1(J).strV2VState(L1(J), L1(I));   % Store V2V data
                    if mod(i,30) == 0 || i == 181% do it every 0.25 second
                        num_iterations = num_iterations + 1; % Count the iterations

                        % Measure execution time for L1 block
                        tic;

                        L1(J).v2vmodel(L1(J), L1(I)); % modelling
                        L1(J).KF(L1(J), L1(I)); % KF
                        L1(J).cred_v2v(L1(J), L1(I)); % cred
                        L1(J).reset_str(L1(J), L1(I)) % reset storage

                        total_time = total_time + toc; % Accumulate total time
                    end
                    L1(J).store_cred(L1(J), L1(I)); % store cred
                end
            end
        end
    end

    % Compute the **average execution time** for both LR & L1
    if num_iterations > 0
        avg_time = total_time / num_iterations;
    else
        avg_time = NaN; % Avoid division by zero
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
            if LR(I).X < L1(1).noisy_X - 50  % Ensure safe distance to merge
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
        TL1 = zeros(length(L1), 1);  % Preallocate
        for j = 1:length(L1)
            if L1(j).OgLnID == 1  % Originally from L1, use noisy_X
                TL1(j) = L1(j).noisy_X - LR(I).X;
            else  % Originally from LR, use normal X
                TL1(j) = L1(j).X - LR(I).X;
            end
        end
        
        Icxp = sum(TL1 > 0);  % Find the first car in L1 ahead of LR
    
        % Handle invalid or out-of-bounds Icxp
        if Icxp == 0 || Icxp > length(L1)
            Icxp = length(L1);  % Default to the last car in L1
        end

        gapAhead = inf;  % Default to large value if no car ahead
        gapBehind = inf; % Default to large value if no car behind
        
        if Icxp > 0 && Icxp <= length(L1)  % Check gap ahead
            if L1(Icxp).OgLnID == 2
                gapAhead = L1(Icxp).X - LR(I).X; % Distance to the car ahead in L1
            else
                gapAhead = L1(Icxp).noisy_X - LR(I).X;
            end
        end
        if Icxp > 1  % Check gap behind
            if L1(Icxp - 1).OgLnID == 2
                gapBehind = L1(Icxp - 1).X - LR(I).X;  % Distance to the car behind in L1
            else
                gapBehind = L1(Icxp - 1).noisy_X - LR(I).X;  % Distance to the car behind in L1
            end
        end
    
        % Skip merging if gaps are insufficient
        % disp(['Car ', num2str(LR(I).ID), ' gap_ahead: ', num2str(gapAhead), ', gap_behind: ', num2str(gapBehind)]);
        % disp(['Current Merge Conditions: gap_ahead > ', num2str(minGapAhead), ' && gap_behind > ', num2str(minGapBehind)]);
        if gapAhead > minGapAhead && gapBehind > minGapBehind
            % disp(['Car ', num2str(LR(I).ID), ' can merge']);
        else
            % disp(['Car ', num2str(LR(I).ID), ' cannot merge: insufficient gaps']);
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
        
        if L1(Icxp).OgLnID == 2
            ah1 = IDM4LC(LR(I), L1(Icxp));  % Acceleration of LR car if it moves into L1
        else
            ah1 = IDM4LCN(LR(I), L1(Icxp));  % Acceleration of LR car if it moves into L1
        end

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
            % disp(['Car ', num2str(LR(I).ID), ' merged']);
            LR(I) = [];  % Remove car from LR
            CarMrg = CarMrg + 1;  % Increment merge count
            Ylc = 2;  % Mark lane change as completed
            break;
        end
    end
end

% Save the results as a `.mat` file
save(fullfile(output_directory, 'Average_Computation_Time_Real_Noise1v6.mat'), 'avg_time');

%% Save L1 and LR data after simulation
save(fullfile(output_directory, 'L1_Real_Noise1v6.mat'), 'L1');
save(fullfile(output_directory, 'LR_Real_Noise1v6.mat'), 'LR');

close(outputVideo); % Finalize and save the video file
PlotCars