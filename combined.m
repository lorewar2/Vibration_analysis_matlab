% Theoritical
% variables
% time difference
start_time = 1;
end_time = 10;

time_small_step = (end_time - start_time) / 10000;
time_full = start_time:time_small_step:end_time;
% mass of the two objects
m1 = 0.672;
m2 = 0.969;
force = 0.105;
kA = 1056;
cA = 482;
current_error = 100000;
previous_error = 100000;
while (1)
    % spring stiffness 
    k1 = kA;
    k2 = kA;
    % spring dampness
    c1 = cA;
    c2 = cA;
    % motor rotational velocity rad per second
    omega = 25; % 733
    
    % force 
    f1 = force * sin(omega * time_full); % 5.3
    f2 = 0;
    
    % initial conditions
    y1 = [0];
    y2 = [0];
    y3 = [0];
    y4 = [0];
    y1dot = y3(end);
    y2dot = y4(end);
    y3dot = ((-(k1 + k2) / m1) * y1(end)) + ((k2 / m2) * y2(end)) - (((c1 + c2)/ m1) * y3(end)) + ((c2/m1) * y4(end)) + f1(1)/m1;
    y4dot = ((k2/m2) * y1(end)) - (((k2)/m2) * y2(end)) + ((c2/m2) * y3(end)) - (((c2)/m2) * y4(end)) + f2/m2;
    x1dotdot = -(((c1 + c2) / m1) * y2(end)) + ((c2 / m1) * y4(end)) - (((k1 + k2)/m1) * y1(end)) + ((k2 / m1) * y3(end)) + (f1(1) / m1);
    x2dotdot = ((c2 / m2) * y2(end)) - ((c2 / m2) * y4(end)) + ((k2 / m2) * y1(end)) - ((k2 / m2) * y3(end)) + (f2 / m2);
    
    % calculate loop
    for i = 1:10000
        % get y by time 
        y1(end + 1) = y1(end) + y1dot(end) * time_small_step;
        y2(end + 1) = y2(end) + y2dot(end) * time_small_step;
        y3(end + 1) = y3(end) + y3dot(end) * time_small_step;
        y4(end + 1) = y4(end) + y4dot(end) * time_small_step;
        % first order differential
        y1dot(end + 1) = y3(end);
        y2dot(end + 1) = y4(end);
        y3dot(end + 1) = ((-(k1 + k2) / m1) * y1(end))+ ((k2 / m2) * y2(end)) - (((c1 + c2)/ m1) * y3(end)) + ((c2/m1) * y4(end)) + f1(i)/m1;
        y4dot(end + 1) = ((k2/m2) * y1(end)) - (((k2)/m2) * y2(end)) + ((c2/m2) * y3(end)) - (((c2)/m2) * y4(end)) + f2/m2;
        % accerleration
        x1dotdot(end + 1) = -(((c1 + c2) / m1) * y2(end)) + ((c2 / m1) * y4(end)) - (((k1 + k2)/m1) * y1(end)) + ((k2 / m1) * y3(end)) + (f1(i) / m1);
        x2dotdot(end + 1) = ((c2 / m2) * y2(end)) - ((c2 / m2) * y4(end)) + ((k2 / m2) * y1(end)) - ((k2 / m2) * y3(end)) + (f2 / m2);
    
    end
    
    % Experimental
    axis = 'x'; 
    % get data for 1
    filename1 = 'top.txt';      % file name
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
    filename2 = 'bottom.txt';  % file name
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
    
    % intrapolate the acceleration to small steps
    intrapolated_acc_1 = interp1(time_1, acc_1, time_full);
    intrapolated_acc_2 = interp1(time_2, acc_2, time_full);
    
    % velocity calculation
    vel_1 = [0];
    vel_2 = [0];
    for c = 2:length(time_full)
        time_gap = time_full(c) - time_full(c-1);
        acc_sum_1 = intrapolated_acc_1(c-1) + intrapolated_acc_1(c);
        acc_sum_2 = intrapolated_acc_2(c-1) + intrapolated_acc_2(c);
        vel_1(end + 1) = 0.5 * time_gap * acc_sum_1;
        vel_2(end + 1) = 0.5 * time_gap * acc_sum_2; 
    end
    
    % displacement calculation
    disp_1 = [0];
    disp_2 = [0];
    for c = 2:length(time_full)
        time_gap = time_full(c) - time_full(c-1);
        vel_sum_1 = vel_1(c-1) + vel_1(c);
        vel_sum_2 = vel_2(c-1) + vel_2(c);
        disp_1(end + 1) = 0.5 * time_gap * vel_sum_1;
        disp_2(end + 1) = 0.5 * time_gap * vel_sum_2; 
    end
    % find the amplitude and wave length
    % theory
    [max_y1, max_y1_index] = max(y1);
    [min_y1, min_y1_index] = min(y1);
    amp_y1 = max_y1 - min_y1;
    wave_len_y1 = time_full(max_y1_index) - time_full(min_y1_index);

    % exp
    [max_disp_1, max_disp_1_index] = max(disp_1);
    [min_disp_1, min_disp_1_index] = min(disp_1);
    amp_disp_1 = max_disp_1 - min_disp_1;
    wave_len_disp_1 = time_full(max_disp_1_index) - time_full(min_disp_1_index);

    previous_error = current_error;
    current_error = immse(amp_y1, amp_disp_1);
    fprintf('%i %i\n', current_error, kA);
    if previous_error < current_error
        break
    end
    break;
end


% plot the graph
figure;
tiledlayout(6,2)
nexttile
plot(time_full, y1)
title('T.Displacement of 1')
xlabel('time (s)')
ylabel('displacement (m)')

nexttile
plot(time_full, disp_1)
title('E.Displacement 1')
xlabel('time (s)')
ylabel('displacement (m)')

nexttile
plot(time_full, y2)
title('T.Displacement of 2')
xlabel('time (s)')
ylabel('displacement (m)')

nexttile
plot(time_full, disp_2)
title('E.Displacement 2')
xlabel('time (s)')
ylabel('displacement (m)')

nexttile
plot(time_full, y3)
title('T.Velocity of 1')
xlabel('time (s)')
ylabel('velocity (ms-1)')

nexttile
plot(time_full, vel_1)
title('E.Velocity 1')
xlabel('time (s)')
ylabel('velocity (ms-1)')


nexttile
plot(time_full, y4)
title('T.Velocity of 2')
xlabel('time (s)')
ylabel('velocity (ms-1)')

nexttile
plot(time_full, vel_2)
title('E.Velocity 2')
xlabel('time (s)')
ylabel('velocity (ms-1)')

nexttile
plot(time_full, x1dotdot)
title('T.Accerleration of 1')
xlabel('time (s)')
ylabel('accerleration (ms-2)')

nexttile
plot(time_full, intrapolated_acc_1)
title('E.Accerleration 1')
xlabel('time (s)')
ylabel('accerleration (ms-2)')

nexttile
plot(time_full, x2dotdot)
title('T.Accerleration of 2')
xlabel('time (s)')
ylabel('accerleration (ms-2)')


nexttile
plot(time_full, intrapolated_acc_2)
title('E.Accerleration 2')
xlabel('time (s)')
ylabel('accerlertion (ms-2)')

