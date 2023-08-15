
clc;
clear all;

% Theoritical
% variables
% time difference
start_time = 1;
end_time = 2;

sampling_frequency = 1000;          % phones sampling frequency
pass_band_frequency = 40;           % cutoff lower frequencies

steps = 1000;
time_small_step = (end_time - start_time) / steps;
time_full = start_time:time_small_step:end_time;

% mass of the two objects
% m1 = 0.672;
% m2 = 0.969;
% m3 = 0.687;

% m1 = 2.394;
% m2 = 1.202;
% m3 = 0.438;

m1 = 0.894;
m2 = 0.987;
m3 = 0.438;

% spring stiffness 
k1 = 61.9835;  %10
k2 = 12.2459; %5
k3 = 10.3005;%10

% spring dampness
c1 = 224.0649;
c2 = 17.2100;
c3 = 7.4193;

% motor rotational velocity rad per second
omega = 30; % 733 30 23.3
force = 0.025; %0.025 20

% force 
f1 = force * sin(omega * time_full); % 5.3 %0.105 %2.235 2.487 2.423 %2.299 0.00025
f2 = 0;
f3 = 0;

% initial conditions
y1 = [0];
y2 = [0];
y3 = [0];
y4 = [0];
y5 = [0];
y6 = [0];

y1dot = y4(end);
y2dot = y5(end);
y3dot = y6(end);

y4dot = ((-(k1 + k2) / m1) * y1(end)) + ((k2 / m1) * y2(end)) - (((c1 + c2)/ m1) * y4(end)) + ((c2 / m1) * y5(end)) + (f1(1)/m1);
y5dot = ((k2 / m2) * y1(end)) - (((k2 + k3)/m2) * y2(end)) + ((k3 / m2) * y3(end)) + ((c2 / m2) * y4(end)) - (((c2 + c3)/m2) * y5(end)) + ((c3 / m2) * y6(end)) + (f2/m2);
y6dot= ((k3 / m3) * y2(end)) - ((k3 / m3) * y3(end)) + ((c3 / m3) * y5(end)) - ((c3 / m3) * y6(end)) + f3/m3;

x1dotdot = -(((c1 + c2) / m1) * y1(end)) + ((c2 / m1) * y3(end)) - (((k1 + k2)/ m1) * y1(end)) + ((k2 / m1) * y4(end)) + (f1(1) / m1);
x2dotdot = ((c2 / m2) * y1(end)) - (((c2 + c3)/ m2) * y3(end)) + ((c3 / m2) * y5(end)) + ((k2 / m2) * y2(end)) - (((k2 + k3)/ m2) * y4(end)) + (f2/m2);
x3dotdot = ((k3 / m3) * y4(end)) - ((k3 / m3) * y6(end)) + ((c3 / m3) * y3(end)) - ((c3 / m3) * y5(end)) + (f3/m3);


% calculate loop
for i = 1:steps
% get y by time 
y1(end + 1) = y1(end) + y1dot(end) * time_small_step;
y2(end + 1) = y2(end) + y2dot(end) * time_small_step;
y3(end + 1) = y3(end) + y3dot(end) * time_small_step;
y4(end + 1) = y4(end) + y4dot(end) * time_small_step;
y5(end + 1) = y5(end) + y5dot(end) * time_small_step;
y6(end + 1) = y6(end) + y6dot(end) * time_small_step;

% first order differential
y1dot(end + 1) = y4(end);
y2dot(end + 1) = y5(end);
y3dot(end + 1) = y6(end);

y4dot(end + 1) = ((-(k1 + k2) / m1) * y1(end)) + ((k2 / m1) * y2(end)) - (((c1 + c2)/ m1) * y4(end)) + ((c2 / m1) * y5(end)) + (f1(i)/m1);
y5dot(end + 1) = ((k2 / m2) * y1(end)) - (((k2 + k3)/m2) * y2(end)) + ((k3 / m2) * y3(end)) + ((c2 / m2) * y4(end)) - (((c2 + c3)/m2) * y5(end)) + ((c3 / m2) * y6(end)) + (f2/m2);
y6dot(end + 1) = ((k3 / m3) * y2(end)) - ((k3 / m3) * y3(end)) + ((c3 / m3) * y5(end)) - ((c3 / m3) * y6(end)) + f3/m3;

% accerleration
x1dotdot(end + 1) = -(((c1 + c2) / m1) * y1(end)) + ((c2 / m1) * y3(end)) - (((k1 + k2)/ m1) * y1(end)) + ((k2 / m1) * y4(end)) + (f1(i) / m1);
x2dotdot(end + 1) = ((c2 / m2) * y1(end)) - (((c2 + c3)/ m2) * y3(end)) + ((c3 / m2) * y5(end)) + ((k2 / m2) * y2(end)) - (((k2 + k3)/ m2) * y4(end)) + (f2/m2);
x3dotdot(end + 1) = ((k3 / m3) * y4(end)) - ((k3 / m3) * y6(end)) + ((c3 / m3) * y3(end)) - ((c3 / m3) * y5(end)) + (f3/m3);
    
end

% Experimental
axis = 'z'; 

% get data for 1
filename1 = 'MODEL_PHONE_01_01.txt';      % file name MODEL_PHONE_01_01
delimiterIn = '\t';
headLinesIn = 0;
imported_data_1 = importdata(filename1, delimiterIn, headLinesIn);
time_1 = imported_data_1(:, 1);
x_acc_1 = imported_data_1(:, 2);
y_acc_1 = imported_data_1(:, 3);
z_acc_1 = imported_data_1(:, 4);
if axis == 'x'
    acc_1 = x_acc_1;
elseif axis == 'y'
    acc_1 = y_acc_1;
elseif axis == 'z'
    acc_1 = z_acc_1;
end

% get data for 2
% variables to change
filename2 = 'MODEL_PHONE_02_01.txt';  % file name MODEL_PHONE_02_01
delimiterIn = '\t';
headLinesIn = 0;
imported_data_2 = importdata(filename2, delimiterIn, headLinesIn);
time_2 = imported_data_2(:, 1);
x_acc_2 = imported_data_2(:, 2);
y_acc_2 = imported_data_2(:, 3);
z_acc_2 = imported_data_2(:, 4);
if axis == 'x'
    acc_2 = x_acc_2;
elseif axis == 'y'
    acc_2 = y_acc_2;
elseif axis == 'z'
    acc_2 = z_acc_2;
end


% get data for 3
% variables to change
filename3 = 'MODEL_PHONE_03_01.txt';  % file name MODEL_PHONE_03_01
delimiterIn = '\t';
headLinesIn = 0;
imported_data_3 = importdata(filename3, delimiterIn, headLinesIn);
time_3 = imported_data_3(:, 1);
x_acc_3 = imported_data_3(:, 2);
y_acc_3 = imported_data_3(:, 3);
z_acc_3 = imported_data_3(:, 4);
if axis == 'x'
   acc_3 = x_acc_3;
elseif axis == 'y'
   acc_3 = y_acc_3;
elseif axis == 'z'
   acc_3 = z_acc_3;
end

% intrapolate the acceleration to small steps
intrapolated_acc_1 = interp1(time_1, acc_1, time_full);
intrapolated_acc_2 = interp1(time_2, acc_2, time_full);
intrapolated_acc_3 = interp1(time_3, acc_3, time_full);

% velocity calculation
vel_1 = [0];
vel_2 = [0];
vel_3 = [0];

for c = 2:length(time_full)
time_gap = time_full(c) - time_full(c-1);
acc_sum_1 = intrapolated_acc_1(c-1) + intrapolated_acc_1(c);
acc_sum_2 = intrapolated_acc_2(c-1) + intrapolated_acc_2(c);
acc_sum_3 = intrapolated_acc_3(c-1) + intrapolated_acc_3(c);
vel_1(end + 1) = 0.5 * time_gap * acc_sum_1;
vel_2(end + 1) = 0.5 * time_gap * acc_sum_2; 
vel_3(end + 1) = 0.5 * time_gap * acc_sum_3; 
end

% displacement calculation
disp_1 = [0];
disp_2 = [0];
disp_3 = [0];

for c = 2:length(time_full)
time_gap = time_full(c) - time_full(c-1);
vel_sum_1 = vel_1(c-1) + vel_1(c);
vel_sum_2 = vel_2(c-1) + vel_2(c);
vel_sum_3 = vel_3(c-1) + vel_3(c);
disp_1(end + 1) = 0.5 * time_gap * vel_sum_1;
disp_2(end + 1) = 0.5 * time_gap * vel_sum_2; 
disp_3(end + 1) = 0.5 * time_gap * vel_sum_3; 
end

theoritical_displacement = y1.';
experimental_displacement = disp_1.';

% fast discrete fourier
N = length(theoritical_displacement);
theoritical_diplacement_fft = fft(theoritical_displacement);
experimental_displacement_fft = fft(experimental_displacement);

% power spectral density
theoritical_diplacement_psdx = (1/(sampling_frequency * N)) * abs(theoritical_displacement).^2;
theoritical_diplacement_psdx(2:end-1) = 2 * theoritical_diplacement_psdx(2:end-1);
t_freq = 0:sampling_frequency/N:sampling_frequency/2;

experimental_displacement_psdx = (1/(sampling_frequency * N)) * abs(experimental_displacement).^2;
experimental_displacement_psdx(2:end-1) = 2 * experimental_displacement_psdx(2:end-1);
e_freq = 0:sampling_frequency/N:sampling_frequency/2;

% % plot the graph
figure;
tiledlayout(3,2)
nexttile
plot(time_full, disp_1)
grid on
title('e time domain plot')
xlabel('time (s)')
ylabel('displacement (m)') 
nexttile
plot(time_full, y1)
grid on
title('t time domain plot')
xlabel('time (s)')
ylabel('displacement (m)')
nexttile
plot(real(experimental_displacement_fft))
grid on
title('e frequency domain plot')
xlabel('time (s)')
ylabel('accer (ms-1)')
nexttile
plot(real(theoritical_diplacement_fft))
grid on
title('t frequency domain plot')
xlabel('time (s)')
ylabel('accer (ms-1)')
nexttile
plot(pow2db(experimental_displacement_psdx))
grid on
title("e Periodogram Using FFT")
xlabel("Frequency (Hz)")
ylabel("Power/Frequency (dB/Hz)")
nexttile
plot(pow2db(theoritical_diplacement_psdx))
grid on
title("t Periodogram Using FFT")
xlabel("Frequency (Hz)")
ylabel("Power/Frequency (dB/Hz)")

