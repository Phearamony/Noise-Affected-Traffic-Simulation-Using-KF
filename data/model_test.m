close all; clear; clc;

%% load mat file

% main vehicle
load("data.mat");

% merging vehicle
% load("C:/Users/monea/OneDrive/Documents/MATLAB/I-80-Emeryville-CA/vehicle-trajectory-data/0500pm-0515pm/main/merging_lane/merging_vehicle_data.mat");

%% adjust variable
% noisy vehicle
t = L1(2, 1).str_v2v_data.v10000.t(50:182);
X_noisy = L1(2, 1).str_v2v_data.v10000.X(50:182);
Y_noisy = L1(2, 1).str_v2v_data.v10000.Y(50:182);
Vel_noisy = L1(2, 1).str_v2v_data.v10000.V(50:182);
Acc_noisy = L1(2, 1).str_v2v_data.v10000.Ac(50:182);

% filter
window_length = 7; % Must be odd
polynomial_order = 3;

X = sgolayfilt(X_noisy, polynomial_order, window_length);
Y = sgolayfilt(Y_noisy, polynomial_order, window_length);
Vel = sgolayfilt(Vel_noisy, polynomial_order, window_length);
Acc = sgolayfilt(Acc_noisy, polynomial_order, window_length);

% real vehicle
X_ = L1(1, 1).str_X(50:182);
Y_ = L1(1, 1).str_Y(50:182);
Vel_ = L1(1, 1).str_V(50:182);
Acc_ = L1(1, 1).str_Ac(50:182);

% X = X_;
% Y = Y_;
% Vel = Vel_;
% Acc = Acc_;

% X_ = X;
% Y_ = Y;
% Vel_ = Vel;
% Acc_ = Acc;


dt = t(2) - t(1); % time differences

% Calculate differences in x and y
dx = diff(X);
dy = diff(Y);

% Calculate the heading angle in radians
theta = atan2(dy, dx);

% Append a NaN or duplicate the last angle to match the original length of data
theta = [theta; theta(end)]; % heading angle
theta = unwrap(theta); % Ensure no sudden jumps

%% initialize A B C D in continuous-time
% Initialize matrices for each time step
num_steps = length(t);
Ac = zeros(3, 3, num_steps); % Preallocate a 3D array for A matrices

% Calculate A matrix for each time step
for k = 1:num_steps
    Ac(:, :, k) = [0, 0, cos(theta(k));
                    0, 0, sin(theta(k));
                    0, 0, 0];
end

Bc = [0; 0; 1];

Cc = eye(3);

Dc = 0;

%% discrete-time
Ad = zeros(3, 3, num_steps);
Bd = zeros(3, 1, num_steps);

% Calculate A matrix for each time step
for k = 1:num_steps
    Ad(:, :, k) = [1, 0, dt * cos(theta(k));
                    0, 1, dt * sin(theta(k));
                    0, 0, 1];

    Bd(:, :, k) = [1/2 * dt^2 * cos(theta(k));
                    1/2 * dt^2 * sin(theta(k));
                    dt];
end

Cd = Cc;
Dd = Dc;

%% Initilize control input and intial values and noise variable
u = Acc;

x = zeros(3, length(t) + 1);
z = zeros(3, length(t));

% initial value
x(:,1) = [X(1); Y(1); Vel(1)];


% value for noise
Bw = Bd; % noise is the input noise

Sw = 0.01; % continuous noise [acc]
Sv = diag([14.27141916, 21.61521545 * 0.8, 0.01]); % continuous noise [x, y, vel]

SigmaW = zeros(3, 3, num_steps); % discrete noise
epsilon = 1e-6; % Small positive value


% for k = 1:num_steps
%     Z = [-Ac(:, :, k) Bw(:, :, k)*Sw*Bw(:, :, k)'
%         zeros(3) Ac(:, :, k)'];
%     C = expm(Z * dt);
%     c12 = C(1:3, 4:6);
%     c22 = C(4:6, 4:6);
%     SigmaW(:, :, k) = c22' * c12 + epsilon * eye(3);
% end

for k = 1:num_steps
    Bw = Bd(:, :, k); % Extract control input matrix for this time step
    SigmaW(:, :, k) = Bw * (0.01 * 1.2) * Bw'; % Process noise covariance
    SigmaW(:, :, k) = SigmaW(:, :, k) + epsilon * eye(size(SigmaW, 1));
end

SigmaV = Sv/dt;

%% Model

for k = 1:length(t)-1 % Loop until the second-to-last element
    x(:,k+1) = Ad(:,:,k) * x(:,k) + Bd(:, :, k) * u(k+1); + chol(SigmaW(:, :, k), 'lower') * randn(3, 1); % Use u(k+1) correctly
    z(:,k) = Cd * x(:,k) + Dd * u(k) + chol(SigmaV, 'lower') * randn(3, 1); % Still use u(k) for the current state
end

% After the loop, handle the last state update and output calculation
x(:, end) = Ad(:,:,end) * x(:, end-1) + Bd(:, :, end) * u(end); % Last state update
z(:, end) = Cd * x(:, end) + Dd * u(end); % Last output

%% plot
figure(1);

% Plot Position
subplot(2, 1, 1);
plot(t, X_, 'b-', 'DisplayName', 'Actual X Position');
hold on;
plot(t, Y_, 'g-', 'DisplayName', 'Actual Y Position');
plot(t, z(1, :), 'r--', 'DisplayName', 'Model X Position');
plot(t, z(2, :), 'k--', 'DisplayName', 'Model Y Position');
xlabel('Time (s)');
ylabel('Position (m)');
title('Vehicle Position');
legend show;
grid on;

% Plot Velocity
subplot(2, 1, 2);
plot(t, Vel_, 'b-', 'DisplayName', 'Actual Velocity');
hold on;
plot(t, z(3, :), 'r--', 'DisplayName', 'Model Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Vehicle Velocity');
legend show;
grid on;

% Adjust figure properties
sgtitle('Vehicle Position and Velocity Comparison');

