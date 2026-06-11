%% Kalman Filter
clear; close all; clc;

% Load data from simulation of system dynamic model
load model_main.mat Ad Bd Cd Dd SigmaW SigmaV dt t u x z X Y Vel
% load model_merge.mat Ad Bd Cd Dd SigmaW SigmaV dt t u x z X Y Vel

% Determine number of states and timesteps
[nx ,nt] = size (x);
[nz, nt_] = size (z);
% Initialize state estimate and covariance
xhat = x(:, 1); SigmaX = zeros(nx ,nx);
% Initialize storage for state / bounds for plotting purposes
xhatstore = zeros (nx ,nt); boundstore = zeros(nx ,nt);
xhatstore(:, 1) = xhat;
% Initialize storage for measurement output
zhatstore = zeros (nz ,nt_);
zhatstore(:, 1) = z(:, 1);
for k = 2:length(t)
    % KF Step 1a: State prediction time update
    xhat = Ad(:,:,k-1) * xhat + Bd(:,:,k-1)*u(k-1); % use prior value of "u"
    % KF Step 1b: Prediction - error covariance time update
    SigmaX = Ad(:,:,k-1) * SigmaX * Ad(:,:,k-1)' + SigmaW(:,:,k-1);
    % KF Step 1c: Predict system output
    zhat = Cd* xhat + Dd*u(k); % use present value of "u"

    % KF Step 2a: Compute estimator matrix
    L = SigmaX * Cd' /( Cd* SigmaX *Cd'+ SigmaV);
    % KF Step 2b: State estimate measurement update
    xhat = xhat + L*(z(:,k) - zhat);
    % KF Step 2c: Estimation - error covariance measurement update
    SigmaX = (eye(nx)-L*Cd)* SigmaX;
    % [ Store estimate and bounds for evaluation / plotting purposes ]
    xhatstore(:,k) = xhat;
    zhatstore(:,k) = zhat;
    boundstore(:,k) = 3* sqrt(diag(SigmaX));
end

%% save mat file for KF
% save model_main_KF.mat xhatstore zhatstore
save model_merge_KF.mat xhatstore zhatstore

%% Plotting the Kalman Filter Results

%% Full Code to Plot Real Data, Model Data, and Kalman Estimates Only (No Errors)

figure;

% Velocity Plot: Real, Model, and Kalman
subplot(3, 1, 1);
plot(t, Vel, 'b-', 'DisplayName', 'Real Velocity');
hold on;
plot(t, z(3, :), 'r--', 'DisplayName', 'Model Velocity');
plot(t, zhatstore(3, :), 'k--', 'DisplayName', 'Kalman Velocity Estimate');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity: Real, Model, and Kalman Estimate');
legend('show');
grid on;

% X Position Plot: Real, Model, and Kalman
subplot(3, 1, 2);
plot(t, X, 'b-', 'DisplayName', 'Real X Position');
hold on;
plot(t, z(1, :), 'r--', 'DisplayName', 'Model X Position');
plot(t, zhatstore(1, :), 'k--', 'DisplayName', 'Kalman X Estimate');
xlabel('Time (s)');
ylabel('X Position (m)');
title('X Position: Real, Model, and Kalman Estimate');
legend('show');
grid on;

% Y Position Plot: Real, Model, and Kalman
subplot(3, 1, 3);
plot(t, Y, 'b-', 'DisplayName', 'Real Y Position');
hold on;
plot(t, z(2, :), 'r--', 'DisplayName', 'Model Y Position');
plot(t, zhatstore(2, :), 'k--', 'DisplayName', 'Kalman Y Estimate');
xlabel('Time (s)');
ylabel('Y Position (m)');
title('Y Position: Real, Model, and Kalman Estimate');
legend('show');
grid on;

% Add a main title for the figure
sgtitle('Comparison of Real Data, Model Data, and Kalman Estimates for X, Y, and Velocity');

%% error

figure;

% Velocity Errors
subplot(3, 1, 1);
plot(t, Vel - zhatstore(3, :)', 'm-', 'DisplayName', 'Error: Real vs Kalman');
hold on;
plot(t, z(3, :) - zhatstore(3, :), 'g-', 'DisplayName', 'Error: Model vs Kalman');
plot(t, Vel - z(3, :)', 'c-', 'DisplayName', 'Error: Real vs Model');
xlabel('Time (s)');
ylabel('Velocity Error (m/s)');
title('Velocity Errors');
legend('show');
grid on;

% X Position Errors
subplot(3, 1, 2);
plot(t, X - zhatstore(1, :)', 'm-', 'DisplayName', 'Error: Real vs Kalman');
hold on;
plot(t, z(1, :) - zhatstore(1, :), 'g-', 'DisplayName', 'Error: Model vs Kalman');
plot(t, X - z(1, :)', 'c-', 'DisplayName', 'Error: Real vs Model');
xlabel('Time (s)');
ylabel('X Position Error (m)');
title('X Position Errors');
legend('show');
grid on;

% Y Position Errors
subplot(3, 1, 3);
plot(t, Y - zhatstore(2, :)', 'm-', 'DisplayName', 'Error: Real vs Kalman');
hold on;
plot(t, z(2, :) - zhatstore(2, :), 'g-', 'DisplayName', 'Error: Model vs Kalman');
plot(t, Y - z(2, :)', 'c-', 'DisplayName', 'Error: Real vs Model');
xlabel('Time (s)');
ylabel('Y Position Error (m)');
title('Y Position Errors');
legend('show');
grid on;

% Add a main title for the error figure
sgtitle('Errors between Real Data, Model Data, and Kalman Estimates for X, Y, and Velocity');