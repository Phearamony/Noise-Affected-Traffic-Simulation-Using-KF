% simple spring-damper system
clear; close all; clc;


% Load data from simulation of system dynamic model
load simOut.mat Ad Bd Cd Dd SigmaV SigmaW dT t u x z
% Determine number of states and timesteps
[nx ,nt] = size (x);
% Initialize state estimate and covariance
xhat = zeros (nx ,1) ; SigmaX = zeros (nx ,nx);
% Initialize storage for state / bounds for plotting purposes
xhatstore = zeros (nx ,nt); boundstore = zeros (nx ,nt);
for k = 2:length(t)
    % KF Step 1a: State prediction time update
    xhat = Ad * xhat + Bd*u(:,k -1); % use prior value of "u"
    % KF Step 1b: Prediction - error covariance time update
    SigmaX = Ad* SigmaX *Ad' + SigmaW ;
    % KF Step 1c: Predict system output
    zhat = Cd* xhat + Dd*u(:,k); % use present value of "u"

    % KF Step 2a: Compute estimator matrix
    L = SigmaX * Cd' /( Cd* SigmaX *Cd' + SigmaV );
    % KF Step 2b: State estimate measurement update
    xhat = xhat + L*(z(:,k) - zhat);
    % KF Step 2c: Estimation - error covariance measurement update
    SigmaX = (eye(nx)-L*Cd)* SigmaX ;
    % [ Store estimate and bounds for evaluation / plotting purposes ]
    xhatstore(:,k) = xhat ;
    boundstore(:,k) = 3* sqrt(diag(SigmaX));
end

figure(1); clf; % Plot the states and estimates
t2 = [t fliplr(t)]; % Prepare for plotting bounds via "fill"
x2 = [xhatstore-boundstore fliplr(xhatstore + boundstore)];
h1 = fill(t2 ,x2 ,'b','FaceAlpha' ,0.5 , 'LineStyle','none'); hold on; grid on
h2 = plot(t,x',t, xhatstore','--'); % Plot true state and its estimate
legend([h2;h1],'True position','True velocity','Position estimate',...
'Velocity estimate','Bounds');
title ('Demonstration of Kalman filter state estimates');
xlabel ('Time (s)'); ylabel ('State (m or m/s)');

figure(2); clf; xerr = x - xhatstore; % Plot state - estimation errors
for k = 1:nx
    subplot(nx ,1,k); % Plot each state 's error in a separate subplot
    fill(t2 ,[-boundstore(k ,:) fliplr(boundstore(k ,:))],'b',...
    'FaceAlpha' ,0.25 , 'LineStyle','none'); hold on; grid on;
    plot(t, xerr(k ,:) ,'b');
    title(sprintf('State %d estimation error with bounds',k));
    xlabel('Time (s)'); ylabel('Error');
end