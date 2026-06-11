%% Credibility Rating
clear; close all; clc;

% Load data from simulation of system dynamic model
load model_main.mat dt t u x z X Y Vel
% load model_merge.mat Ad Bd Cd Dd SigmaW SigmaV dt t u x z X Y Vel

% Load data from KF
load model_main_KF.mat xhatstore zhatstore
% load model_merge_KF.mat xhatstore zhatstore

size = 50; % 50 windows size
errors = [];

% make sure it is not 0
buffer = [1e-5; 1e-5; 1e-5];

threshold = 0.65; % Define threshold for trustworthiness

cred_x_store = [];
cred_y_store = [];
cred_v_store = [];
cred_store = [];

% weight for x y vel
wx = 1/3;
wy = 1/3;
wv = 1/3;

% Initialize storage for results
results = table(); % Empty table to store results

for i = 1:length(t)
    % calculate the error
    current_error = [X(i) - zhatstore(1,i); Y(i) - zhatstore(2, i); Vel(i) - zhatstore(3, i)];
    
    % add the current error to the windowed errors array
    errors = [errors, current_error];
    if length(errors) > size
        errors = errors(:, 2:end); % keep the errors size 50
    end
    % Update sigma_squared
    sigma_squared = [var(errors(1, :)); var(errors(2,:)); var(errors(3, :))] + buffer;
    
    
    % credibility
    cred_x = exp(-current_error(1)^2 / sigma_squared(1));
    cred_x_store = [cred_x_store, cred_x];

    cred_y = exp(-current_error(2)^2 / sigma_squared(2));
    cred_y_store = [cred_y_store, cred_y];

    cred_v = exp(-current_error(3)^2 / sigma_squared(3));
    cred_v_store = [cred_v_store, cred_v];

    cred = wx * cred_x + wy * cred_y + wv * cred_v;
    cred_store = [cred_store, cred];

    % Determine if trust is True or False
    trust = cred >= threshold;

    % Append the current time, credibility, and trust to results
    results = [results; table(t(i), cred, trust)];
end

% Assign column names for clarity
results.Properties.VariableNames = {'Time', 'Credibility', 'Trust'};

%% Plot

figure;

% Velocity Plot: Real, and Kalman
subplot(4, 1, 1);
plot(t, Vel, 'b-', 'DisplayName', 'Real Velocity');
hold on;
plot(t, zhatstore(3, :), 'k--', 'DisplayName', 'Kalman Velocity Estimate');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity: Real, and Kalman Estimate');
legend('show');
grid on;

% X Position Plot: Real, and Kalman
subplot(4, 1, 2);
plot(t, X, 'b-', 'DisplayName', 'Real X Position');
hold on;
plot(t, zhatstore(1, :), 'k--', 'DisplayName', 'Kalman X Estimate');
xlabel('Time (s)');
ylabel('X Position (m)');
title('X Position: Real, and Kalman Estimate');
legend('show');
grid on;

% Y Position Plot: Real, and Kalman
subplot(4, 1, 3);
plot(t, Y, 'b-', 'DisplayName', 'Real Y Position');
hold on;
plot(t, zhatstore(2, :), 'k--', 'DisplayName', 'Kalman Y Estimate');
xlabel('Time (s)');
ylabel('Y Position (m)');
title('Y Position: Real,, and Kalman Estimate');
legend('show');
grid on;

% Crediblity Plot
subplot(4, 1, 4);
plot(t, results.Credibility, 'b-', 'DisplayName', 'Crediblity');
hold on;
plot(t, results.Trust, 'k--', 'DisplayName', 'TruthWorthy');
xlabel('Time (s)');
ylabel('Credibility');
title('Crediblity');
legend('show');
grid on;