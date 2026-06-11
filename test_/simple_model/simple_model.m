% simple spring-damper system
clear; close all; clc;

m = 50; b = 2; k = 4;
A = [0 1
    -k/m -b/m];
B = [0
    1/m];
C = [1 0];
D = 0;

dT = 0.1;
Ad = expm(A * dT);
Bd = A \ (Ad - eye(2)) * B;
Cd = C;
Dd = D;

Sw = 0.01; Sv = 1e-5;
Bw = B; % assume the sensor noise is the same as input
Z = [-A Bw*Sw*Bw'
    zeros(2) A'];
C = expm(Z*dT);
c12 = C(1:2, 3:4);
c22 = C(3:4, 3:4);
SigmaW = c22' * c12;
SigmaV = Sv/dT;

t = 0:dT:100;
u = sin(2 * pi *(1/10) *t); % assume the input is sine wave

x = zeros(2, length(t) + 1);
z = zeros(1, length(t));

for k = 1:length(t)
    x(:,k+1) = Ad * x(:,k) + Bd * u(k) + chol(SigmaW, 'lower') * randn(2, 1);
    z(k) = Cd * x(:,k) + Dd * u(k) + chol(SigmaV, 'lower') * randn(1);
end
x = x(:, 1:length(t)); % Crop states to same length as t

figure(1);
plot(t, x(1, :)); % position

figure(2);
plot(t, x(2, :)); % velocity

figure(3);
plot(t, z); % noisy position output measurement


% save for KF
save simOut.mat Ad Bd Cd Dd SigmaW SigmaV dT t u x z

