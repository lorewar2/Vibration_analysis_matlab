clc;
clear all;


% optimizer start
options = optimoptions('fmincon',...
    'PlotFcn','optimplotfvalconstr',...
    'Display','iter');
% k1 k2 k3 c1 c2 c3
x0 = [10 10 10 10 10 10];  %200 150 120 80 20 10
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];


[x,fval] = fmincon(@get_loss_from_criteria, x0, A, b, Aeq, beq, lb, ub, @limit, options)



function [c,ceq] = limit(x)
    c = x(1) + x(2) + x(3) + x(4) + x(5) + x(6)- 10000;
    ceq = [ ];
end

function f = get_loss_from_criteria(x)

% Theoritical
% variables
% time difference
k1 = x(1);
    k2 = x(2);
    k3 = x(3);
    c1 = x(4);
    c2 = x(5);
    c3 = x(6);
    % parameters
    start_time = 1;
    end_time = 10;  %60
    time_small_step = (end_time - start_time) / 10000;
    time_full = start_time:time_small_step:end_time;
    axis = 'z';
    % rotational velocity rads-1
    omega = 23.3; % 733
    % force 
    force = 20;    %20
    f1 = force * sin(omega * time_full); % 5.3
    f2 = 0;
    f3 = 0;
    % mass of the two objects
    m1 = 0.672;
    m2 = 0.969;
    m3 = 0.687;
    % get the experimental values first (unchanged)
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

    % get data for 3
    % variables to change
    filename3 = 'top.txt';  % file name
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

    % get the theoritial values
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
    for i = 1:10000
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

    % find the amplitude and wave length
    % theory
    [max_y1, ~] = max(y1);
    [min_y1, ~] = min(y1);
    amp_y1 = max_y1 - min_y1;
   
    % exp
    [max_disp_1, ~] = max(disp_1);
    [min_disp_1, ~] = min(disp_1);
    amp_disp_1 = max_disp_1 - min_disp_1;

    % find a wave in theoritical and experimental
    % find the index of the time required
    index = 0;
    for time = time_full
        index = index + 1;
        if time > 5
            break;
        end
    end
    % theoritical wave start and end
    [t_wstart, t_wend] = get_wave_start_end(y1, index);
    
    % experimental wave start and end
    [e_wstart, e_wend] = get_wave_start_end(disp_1, index);
    t_frequency = 1 / (time_full(t_wend) - time_full(t_wstart));
    e_frequency = 1 / (time_full(e_wend) - time_full(e_wstart));

    fprintf('theory frequency %x \n', t_frequency);
    fprintf('experimental frequency %x \n', e_frequency);
    
    mse = sum((amp_y1 - amp_disp_1).^2) / length(amp_y1);

    f = mse;
    
end

%helper functions

function [wstart, wend] = get_wave_start_end(array, index)
    going_down = 1;
    prev = array(index);
    index = index + 1;
    current = array(index);
    if prev > current
        going_down = 0;
    end
    if going_down
        % find the start +ve to -ve
        while(1)
            prev = array(index);
            index = index + 1;
            current = array(index);
            if prev > current
                break
            end
        end
        % find the start -ve to +ve
        while(1)
            prev = array(index);
            index = index + 1;
            current = array(index);
            if prev < current
                break
            end
        end
        wstart = index;
        % go till +ve to -ve
        while(1)
            prev = array(index);
            index = index + 1;
            current = array(index);
            if prev > current
                break
            end
        end
        % find the end -ve to +ve
        while(1)
            prev = array(index);
            index = index + 1;
            current = array(index);
            if prev < current
                break
            end
        end
        wend = index;
    else
        % find the start -ve to +ve
        while(1)
            prev = array(index);
            index = index + 1;
            current = array(index);
            if prev < current
                break
            end
        end
        wstart = index;
        % go till +ve to -ve
        while(1)
            prev = array(index);
            index = index + 1;
            current = array(index);
            if prev > current
                break
            end
        end
        % find the end -ve to +ve
        while(1)
            prev = array(index);
            index = index + 1;
            current = array(index);
            if prev < current
                break
            end
        end
        wend = index;
    end
end
