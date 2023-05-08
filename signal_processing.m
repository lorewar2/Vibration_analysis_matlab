% variables to change
sampling_frequency = 1000;          % phones sampling frequency
pass_band_frequency = 40;           % cutoff lower frequencies
filename = 'rigid_data_1.txt';      % file name
axis = 'x';                         % axis to check

% get data from file
delimiterIn = '\t';
headLinesIn = 0;
imported_data = importdata(filename, delimiterIn, headLinesIn);

% data in arrays
time = imported_data(:, 1);
x_acc = imported_data(:, 2);
y_acc = imported_data(:, 3);
z_acc = imported_data(:, 4);
if axis == 'x'
    acc = x_acc;
elseif axis == 'y'
    acc = y_acc;
elseif axis == 'z'
    acc = z_acc;
end

% intrapolate the acceleration to small steps
time_small_step = 0:0.001:1;
intrapolated_acc = interp1(time, acc, time_small_step);

% velocity calculation
vel = [0];
for c = 2:length(time_small_step)
    time_gap = time_small_step(c) - time_small_step(c-1);
    acc_sum = intrapolated_acc(c-1) + intrapolated_acc(c);
    vel(end + 1) = 0.5 * time_gap * acc_sum; 
end

% displacement calculation
disp = [0];
for c = 2:length(time_small_step)
    time_gap = time_small_step(c) - time_small_step(c-1);
    vel_sum = vel(c-1) + vel(c);
    disp(end + 1) = 0.5 * time_gap * vel_sum; 
end

% plot the graph
figure;
tiledlayout(3,1)
nexttile
plot(time_small_step, intrapolated_acc)
title('Acceleration intrapolated, 1000 steps per second')
xlabel('time (s)')
ylabel('accer (ms-2)')
nexttile
plot(time_small_step, vel)
title('Velocity graph')
xlabel('time (s)')
ylabel('velocity (ms-1)')
nexttile
plot(time_small_step, disp)
title('Displacement graph')
xlabel('time (s)')
ylabel('displacement (m)')

[max_disp, max_disp_index] = max(disp);
acc_at_max_disp = intrapolated_acc(max_disp_index);
fprintf('max_displacement = %im \nacc at max dis = %im\n', max_disp, acc_at_max_disp)



%
% % high pass filter
% passed_acc = highpass(acc, pass_band_frequency, sampling_frequency);
% % fast discrete fourier
% N = length(acc);
% xdft_acc = fft(passed_acc);
% xdft_acc = xdft_acc(1:N/2+1);
% 
% % power spectral density
% psdx = (1/(sampling_frequency*N)) * abs(xdft_acc).^2;
% psdx(2:end-1) = 2*psdx(2:end-1);
% freq = 0:sampling_frequency/N:sampling_frequency/2;
% 
% % plot the graph
% figure;
% tiledlayout(3,1)
% nexttile
% plot(time, acc)
% grid on
% title('time domain plot')
% xlabel('time (s)')
% ylabel('accer (ms-1)')
% nexttile
% plot(time, passed_acc)
% grid on
% title('high-pass filtered time domain plot')
% xlabel('time (s)')
% ylabel('accer (ms-1)')
% nexttile
% plot(freq,pow2db(psdx))
% grid on
% title("Periodogram Using FFT")
% xlabel("Frequency (Hz)")
% ylabel("Power/Frequency (dB/Hz)")