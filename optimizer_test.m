
% optimizer start
options = optimoptions('fmincon',...
    'PlotFcn','optimplotfvalconstr',...
    'Display','iter');
% k1 k2 c1 c2
x0 = [10 10 10 10];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];

[x,fval] = fmincon(@get_loss_from_criteria, x0, A, b, Aeq, beq, lb, ub, @limit, options)

function [c,ceq] = limit(x)
    c = x(1) + x(2) + x(3) + x(4) - 10000;
    ceq = [ ];
end

function f = get_loss_from_criteria(x)
    k1 = x(1);
    k2 = x(2);
    c1 = x(3);
    c2 = x(4);
    % parameters
    start_time = 1;
    end_time = 10;
    time_small_step = (end_time - start_time) / 10000;
    time_full = start_time:time_small_step:end_time;
    axis = 'x';
    % rotational velocity rads-1
    omega = 23.3; % 733
    % force 
    force = 20;
    f1 = force * sin(omega * time_full); % 5.3
    f2 = 0;
    % mass of the two objects
    m1 = 0.672;
    m2 = 0.969;
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
    % get the theoritial values
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
    % get the relative points from 0
    t_relative = get_relative_points(y1, t_wstart, t_wend);
    e_relative = get_relative_points(disp_1, e_wstart, e_wend);
    fprintf('theory wave length %x \n', time_full(t_wend) - time_full(t_wstart));
    fprintf('experimental wave length %x \n', time_full(e_wend) - time_full(e_wstart));
    
    % calculate point wise distance
    error_sum = 0;
    for i = 1:length(t_relative)
        diff = t_relative(i) - e_relative(i);
        error_sum = error_sum + (diff ^ 2);
    end
    error_mean = error_sum / length(t_relative);
    f = error_mean;
end
%optimizer end

%helper functions
function relative_points = get_relative_points(array, index_start, index_end)
    relative_points = [];
    lowest_point = array(index_start);
    for i = array(index_start:index_end)
        relative_points(end + 1) = i - lowest_point;
    end
end

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
