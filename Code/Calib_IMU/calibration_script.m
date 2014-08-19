
filename = '14_08_2014_MAV051_compressed.txt';

fileid = fopen(filename);
textFromFile = textscan(fileid,'%s');%,'HeaderLines',2,'Delimiter','\t');

name = textFromFile{1,1}(1:23);

A = dlmread(filename,'\t',1,0);

fclose('all');

%% Accelerations
g = 9.81;

accrawx = 2;
accrawy = 4;
accrawz = 6;

%accrawinter = 1:size(A,1);
indexmax = find(A(:,1)>1000);
accrawinter = indexmax;

figure
hold on
plot(A(accrawinter,1),A(accrawinter,accrawx))
plot(A(accrawinter,1),A(accrawinter,accrawy),'r')
plot(A(accrawinter,1),A(accrawinter,accrawz),'g')
xlabel('Time')
ylabel('Acceleration Intensity')
legend('X', 'Y', 'Z')
hold off

figure
accmatrix = [A(accrawinter,accrawx) A(accrawinter,accrawy) A(accrawinter,accrawz)];

[b_acc,D_acc] = calibrate_acc_standalone(accmatrix,1);

accscaledx = D_acc(1,1) * (A(accrawinter,accrawx) - b_acc(1));
accscaledy = D_acc(2,2) * (A(accrawinter,accrawy) - b_acc(2));
accscaledz = D_acc(3,3) * (A(accrawinter,accrawz) - b_acc(3));

figure
hold on
plot(A(accrawinter,1),accscaledx)
plot(A(accrawinter,1),accscaledy,'r')
plot(A(accrawinter,1),accscaledz,'g')
xlabel('Time')
ylabel('Intensity')
legend('X', 'Y', 'Z')
hold off

scale_acc = g./diag(D_acc);
display(scale_acc)
% display(b_acc)

% accx = D(1,1)*(accx_raw - b(1));

% Maveric code
% sf = 1/scalefactor
% be = biais * sf
% scaleddata = rawdata * sf - be
%            = rawdata * sf - biais * sf
%            = (rawdata - biais ) * sf

% scaleddata = (rawdata - be) * sf

%% Magnetometer

magrawx = 3;
magrawy = 5;
magrawz = 7;

%magrawinter = 1:size(A,1);
indexmax = find(A(:,1)>600);
magrawinter = indexmax;

figure
hold on
plot(A(magrawinter,1),A(magrawinter,magrawx))
plot(A(magrawinter,1),A(magrawinter,magrawy),'r')
plot(A(magrawinter,1),A(magrawinter,magrawz),'g')
xlabel('Time')
ylabel('Magnetometer Intensity')
legend('X', 'Y', 'Z')
hold off

magmatrix = [A(magrawinter,magrawx) A(magrawinter,magrawy) A(magrawinter,magrawz)];

[b_mag,D_mag] = calibrate_acc_standalone(magmatrix,2);

magscaledx = D_mag(1,1) * (A(magrawinter,magrawx) - b_mag(1));
magscaledy = D_mag(2,2) * (A(magrawinter,magrawy) - b_mag(2));
magscaledz = D_mag(3,3) * (A(magrawinter,magrawz) - b_mag(3));

figure
hold on
plot(A(magrawinter,1),magscaledx)
plot(A(magrawinter,1),magscaledy,'r')
plot(A(magrawinter,1),magscaledz,'g')
xlabel('Time')
ylabel('Intensity')
legend('X', 'Y', 'Z')
hold off


scale_mag = 1./diag(D_mag);
display(scale_mag)
% display(b_mag)

%frontdirz = mean(magscaledz(A(:,1)>0&A(:,1)<3900));
%display(frontdirz)