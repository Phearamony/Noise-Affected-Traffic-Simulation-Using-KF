close all; clear; clc;

%% load mat file

% main vehicle
load("C:/Users/monea/OneDrive/Documents/MATLAB/I-80-Emeryville-CA/vehicle-trajectory-data/0500pm-0515pm/main/merging_lane/main_vehicle_data.mat");

% merging vehicle
% load("C:/Users/monea/OneDrive/Documents/MATLAB/I-80-Emeryville-CA/vehicle-trajectory-data/0500pm-0515pm/main/merging_lane/merging_vehicle_data.mat");

%% adjust variable
% main vehicle
t = main_vehicle_data.Time;
X = main_vehicle_data.X;
Y = main_vehicle_data.Y;
Vel = main_vehicle_data.Velocity;
Acc = main_vehicle_data.Acceleration;

% merging vehicle
% t = merging_vehicle_data.Time;
% X = merging_vehicle_data.X;
% Y = merging_vehicle_data.Y;
% Vel = merging_vehicle_data.Velocity;
% Acc = merging_vehicle_data.Acceleration;

dt = t(2) - t(1); % time differences

% Calculate differences in x and y
dx = diff(X);
dy = diff(Y);

% Calculate the heading angle in radians
theta = atan2(dy, dx);

% Append a NaN or duplicate the last angle to match the original length of data
theta = [theta; theta(end)]; % heading angle

%% initialize A B C D in discrete-time
% Initialize matrices for each time step
num_steps = length(t);
A = zeros(3, 3, num_steps); % Preallocate a 3D array for A matrices
B = zeros(3, 1, num_steps);

% Calculate A matrix for each time step
for k = 1:num_steps
    A(:, :, k) = [1, 0, dt * cos(theta(k));
                  0, 1, dt * sin(theta(k));
                  0, 0, 1];

    B(:, :, k) = [1/2 * dt^2 * cos(theta(k))
                  1/2 * dt^2 * sin(theta(k))
                  dt];
end

C = [1 0 0
    0 1 0
    0 0 1];

D = 0;

u = Acc;

Bw = B;

x = zeros(3, length(t) + 1);
z = zeros(3, length(t));

% initial value

x(:,1) = [X(1); Y(1); Vel(1)];

%% Model

for k = 1:length(t)-1 % Loop until the second-to-last element
    x(:,k+1) = A(:,:,k) * x(:,k) + B(:,:,k) * u(k+1); % Use u(k+1) correctly
    z(:,k) = C * x(:,k) + D * u(k); % Still use u(k) for the current state
end

% After the loop, handle the last state update and output calculation
x(:, end) = A(:,:,end) * x(:, end-1) + B(:,:,end) * u(end); % Last state update
z(:, end) = C * x(:, end) + D * u(end); % Last output

%% plot
figure(1);

% Plot Position
subplot(2, 1, 1);
plot(t, X, 'b-', 'DisplayName', 'Actual X Position');
hold on;
plot(t, Y, 'g-', 'DisplayName', 'Actual Y Position');
plot(t, z(1, :), 'r--', 'DisplayName', 'Model X Position');
plot(t, z(2, :), 'k--', 'DisplayName', 'Model Y Position');
xlabel('Time (s)');
ylabel('Position (m)');
title('Vehicle Position');
legend show;
grid on;

% Plot Velocity
subplot(2, 1, 2);
plot(t, Vel, 'b-', 'DisplayName', 'Actual Velocity');
hold on;
plot(t, z(3, :), 'r--', 'DisplayName', 'Model Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Vehicle Velocity');
legend show;
grid on;

% Adjust figure properties
sgtitle('Vehicle Position and Velocity Comparison');


%% Kalman Filter

% Determine number of states and timesteps
[nx ,nt] = size (x);
% Initialize state estimate and covariance
xhat = [X(1); Y(1); Vel(1)]; SigmaX = zeros(nx ,nx);
% Initialize storage for state / bounds for plotting purposes
xhatstore = zeros (nx ,nt); boundstore = zeros(nx ,nt);
for k = 2:length(t)
    % KF Step 1a: State prediction time update
    xhat = A(:,:,k-1) * xhat + B(:,:,k-1)*u(k-1); % use prior value of "u"
    % KF Step 1b: Prediction - error covariance time update
    SigmaX = A(:,:,k-1) * SigmaX * A(:,:,k-1)' ;
    % KF Step 1c: Predict system output
    zhat = C* xhat + D*u(k); % use present value of "u"

    % KF Step 2a: Compute estimator matrix
    L = SigmaX * C' /( C* SigmaX *C');
    % KF Step 2b: State estimate measurement update
    xhat = xhat + L*(z(:,k) - zhat);
    % KF Step 2c: Estimation - error covariance measurement update
    SigmaX = (eye(nx)-L*C)* SigmaX;
    % [ Store estimate and bounds for evaluation / plotting purposes ]
    xhatstore(:,k) = xhat;
    boundstore(:,k) = 3* sqrt(diag(SigmaX));
end

%% Plotting the Kalman Filter Results

