% from logs of slow rotations without linear accelerations, this script 
% extracts the biases and scale factors of the accelerometer measurements.
% It uses the measurement of gravity in many orientations to fit an
% ellipsoid to the point cloud.
% 
% Implementation based on the paper : 
% Timo Pylvänäinen, "Automatic and adaptive calibration of 3D field sensors"
%
% formula to get real accelerations (accx) from raw measurements (accx_raw):
%   accx = D(1,1)*(accx_raw - b(1));
%
% Author: Adrien Briod
% Date: 22.12.11
function [b,D] = calibrate_acc_standalone(acc,type)

g = 9.81;

if size(acc,1) < size(acc,2)
    acc = acc';
end

if type == 1
    ax = acc(:,1)/g;
    ay = acc(:,2)/g;
    az = acc(:,3)/g;
else
    ax = acc(:,1);
    ay = acc(:,2);
    az = acc(:,3);
end

sigma = [ax.^2 + ay.^2 - 2*az.^2 ...
    ax.^2 - 2*ay.^2 + az.^2 ...
    4*ax.*ay ...
    2*ax.*az ...
    2*ay.*az ...
    ax ...
    ay ...
    az ...
    ones(length(ax),1)
    ];
e = ax.^2 + ay.^2 + az.^2;

Sls = inv(sigma'*sigma)*sigma'*e;

U = Sls(1);
V = Sls(2);
M = Sls(3);
N = Sls(4);
P = Sls(5);
Q = Sls(6);
R = Sls(7);
S = Sls(8);
T = Sls(9);

A = [1-U-V -2*M -N; -2*M 1-U+2*V -P; -N -P 1+2*U-V];
b = inv(A)/2*[Q; R; S];
c = T + b'*A*b;

B = 1/c*A;
d11 = sqrt(B(1,1));
d21 = 0;
d31 = 0;
d12 = B(1,2)/sqrt(B(1,1));
d22 = sqrt(B(2,2) - B(1,2)^2/B(1,1));
d32 = 0;
d13 =  B(1,3)/d11;
d23 = (B(2,3)-d12*d13)/d22;
d33 = sqrt(B(3,3)-d13^2-d23^2);
D = [d11 d12 d13;
     d21 d22 d23;
     d31 d32 d33];
 
% displays the ellipse to prove it works :

% simulates accelerations around a 9.81 sphere :
[sx, sy, sz] = ellipsoid(0,0,0,1,1,1,30);

for i = 1:size(sx,1)
    for j = 1:size(sx,2)
        u = [sx(i,j); sy(i,j); sz(i,j)];
        m = inv(D)*u + b;
        ex(i,j) = m(1);
        ey(i,j) = m(2);
        ez(i,j) = m(3);
    end
end

% scales b correctly
if type == 1
    b = b*g;
end

%%
%close all
scatter3(ax(1:10:end),ay(1:10:end),az(1:10:end));
set(gca,'DataAspectRatio',[1 1 1])
hold on
surf(ex, ey, ez,'AlphaData',0.5,'FaceColor',[.5 .5 .5],'AmbientStrength',0.4)
alpha(.8)
legend('accelerometer readings','fitted ellipsoid')
hold off