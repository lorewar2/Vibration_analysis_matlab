% variables
% time difference
time_small_step = 0.01;
time_full = 0:0.001:10;
% mass of the two objects
m1 = 15;
m2 = 10;
% spring stiffness 
k1 = 20;
k2 = 30;
% spring dampness
c1 = 10;
c2 = 10;

% motor rotational velocity rad per second
omega = 20;

% force 
f1 = 50 * sin(omega * time_full);
f2 = 0;

% initial conditions
y1 = [0];
y2 = [0];
y3 = [0];
y4 = [0];
y1dot = y3(end);
y2dot = y4(end);
y3dot = ((-(k1 + k2) / m1) * y1(end))+ ((k2 / m2) * y2(end)) - (((c1 + c2)/ m1) * y3(end)) + ((c2/m1) * y4(end)) + f1(1)/m1;
y4dot = ((k2/m2) * y1(end)) - (((k2)/m2) * y2(end)) + ((c2/m2) * y3(end)) - (((c2)/m2) * y4(end)) + f2/m2;

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

end

% plot the graph
figure;
tiledlayout(4,1)
nexttile
plot(time_full, y1)
title('Displacement of x1')
xlabel('time (s)')
ylabel('displacement (m)')
nexttile
plot(time_full, y2)
title('Displacement of x2')
xlabel('time (s)')
ylabel('displacement (m)')
nexttile
plot(time_full, y3)
title('velocity of x1')
xlabel('time (s)')
ylabel('velocity (ms-1)')
nexttile
plot(time_full, y4)
title('velocity of x2')
xlabel('time (s)')
ylabel('velocity (ms-1)')



