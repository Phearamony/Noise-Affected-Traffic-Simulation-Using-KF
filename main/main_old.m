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

%% Congested
for I=1:30
    L1(end+1,1)=Car(20000+I,1,L1(end).X-70);
    if(I<25)
        LR(end+1,1)=Car(30000+I,2,LR(end).X-100);
    end
end

plt=[];
NextCarIDLane=[L1(end).ID+1; LR(end).ID+1];

hFig=figure;
set(hFig, 'position', [20,300,1500,120]);
ti=cputime;

CarInPer10s=0;

 %% Main
for i=1:2401

    if(mod(i,2)==1) %% Control how frequently you want to display vehicles.
        SimPlots3Lane3n;
    end

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
    end

    for I=2:length(LR)
        Xp=LR(I).X;
        LR(I).X=LR(I).fdx(LR(I));
        LR(I).V=LR(I).fdv(LR(I));
    end

    %% MOBIL Lane Change
    CnL = length(LR);  % Count of cars in LR (merging lane)
    Ylc = 0;  % Lane change flag
    for I = 2:min(CnL - 2, 10)  % Check for the first few cars in LR min(CnL - 2, 10)
        if(I > CnL - 1|| LR(I).X < 1000)  % Stop if car is too far or out of range
            break; 
        end
    
        % Calculate the gap between the car in LR and the cars in L1
        L1(1:end)
        Icxp = sum(TL1 > 0);  % Find the car in L1 that is in front of the car in LR
        
        if(Icxp <= 0)
            Icxp = 1; 
        end
        if(length(L1) <= Icxp)
            break;  % Stop if there’s no car in L1 ahead
        end
        if(Ylc > 0)
            Ylc = max(Ylc - 1, 0);  % Decrease Ylc as lane change progresses
            continue;  % Skip iteration if lane change is happening
        end

        % Compute accelerations using the IDM model
        ah = IDM4LC(LR(I), LR(I - 1));  % Acceleration of car in LR with respect to car behind
        ao = IDM4LC(LR(I + 1), LR(I));  % Acceleration of car in LR with respect to the car ahead
        an = IDM4LC(L1(Icxp + 1), L1(Icxp));  % Acceleration of the car in L1 ahead of the gap
    
        ah1 = IDM4LC(LR(I), L1(Icxp));  % Acceleration of car in LR if it moves into L1
        ao1 = IDM4LC(LR(I + 1), LR(I - 1));  % Acceleration of the car in LR relative to L1 car behind
        an1 = IDM4LC(L1(Icxp + 1), LR(I));  % Acceleration of the car in L1 ahead of LR
    
        % If conditions for merging are met, perform the lane change
        if(ah < ah1 + 2.0 && an1 > -6.0)  % Merging condition
            LR(I).LnID = 1;  % Move car from LR to L1 (set lane ID to 2 for L1)
            LR(I).Y = LR(I).Y - 0.15;  % Adjust Y position to simulate lane change
            L1(Icxp + 2:end + 1) = L1(Icxp + 1:end);  % Shift cars in L1 to make space
            L1(Icxp + 1) = LR(I);  % Add car from LR to L1
            LR(I) = [];  % Remove car from LR
            CarMrg = CarMrg + 1;  % Increment merge count
            Ylc = 2;  % Mark that a lane change is completed
        end
    end
end


%% movie2avi(M, 'TrafficFlow7.avi', 'fps',4,'quality',75);

