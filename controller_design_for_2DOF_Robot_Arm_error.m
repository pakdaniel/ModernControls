close all;
format long;
%% ARM PARAMETERS 
global m1 m1_real m2 m2_real l1 l1_real l2 l2_real g u_max x_prev x_history x_d_history t_history u_d_history u_fb_history u_noise_history LineWidth t_history_offset integral_error t_old
m1 = 1; %kg
m2 = 1; %kg
l1 = 1; %m
l2 = 1; %m

m1_real = 1.1;
m2_real = 1.1;
l1_real = 1.1;
l2_real = 1.1;

g = 9.8; %N/kg
u_max = 50; %Nm
LineWidth = 1.3;

% PID CONTROL PARAMETER
integral_error = [0, 0];
t_old = 0;

%% SIMULATION OF THE PATH TRACING
% PARAMETERS
start_pos = [0, 1];
end_pos = [sin(pi/3), -sin(pi/6)];
Ti = 0; %s
Tf = 3; %s
dT = Tf-Ti;
x0 = [cal_angle(start_pos, [pi/3, pi/3, 0, 0]), 0, 0];
t_history_offset =  0;

x_d_history = x0;
x_history = x0;
x_prev = x0;
t_history = 0;
u_d_history = [];
u_noise_history = [];
u_fb_history = [];

% SIMULATION
options = odeset('RelTol',1e-5,'AbsTol',1e-5,'OutputFcn',@odeplot);
[ts1, xs1] = ode45(@(t,y) eom(t, y, start_pos, end_pos, dT), Ti:.01:Tf, x0);
start_pos = [sin(pi/3), -sin(pi/6)];
end_pos = [-sin(pi/3), -sin(pi/6)];
% Ti = Tf; %s
% Tf = Ti+dt; %s
x0 = x_history(end,:);%[cal_angle(start_pos, xs(end,:)), 0, 0];
t_history_offset = t_history(end);

[ts2, xs2] = ode45(@(t,y) eom(t, y, start_pos, end_pos, dT), Ti:.01:Tf, x0);

start_pos = [-sin(pi/3), -sin(pi/6)];
end_pos = [0, 1];
% Ti = Tf; %s
% Tf = Ti+dt; %s
x0 = x_history(end,:); %[cal_angle(start_pos, xs(end,:)), 0, 0];
t_history_offset =  t_history(end);

[ts3, xs3] = ode45(@(t,y) eom(t, y, start_pos, end_pos, dT), Ti:.01:Tf, x0);


% %% Interpolate Data
% t_interp = [ts1; ts2(2:end)+dT; ts3(2:end)+2*dT];
% for i = 2:size(x_history,1)
%     if t_history(i-1) == t_history(i)
%         t_history(i) = t_history(i) + .000000000000001;
%         x_history(i,:) = x_history(i,:) + .000000000000001;
%         x_d_history(i,:) = x_d_history(i,:) + .000000000000001;
%     end
% end
% for i = 1:size(x_history,2)
%     x_history(:,i) = interp1(t_history,x_history(:,i),t_interp);
%     x_d_history(:,i) = interp1(t_history,x_d_history(:,i),t_interp);
% end
% t_history = t_interp;
% x_history = [xs1; xs2(2:end,:); xs3(2:end,:)];


%% CREATE PLOTTABLE DATA
error = x_history - x_d_history;
u_fb_history = u_noise_history - u_d_history;

% Make u vectors the same length as the t_history vector. Set first element
% equal to the second element as an approximation
u_d_history = [u_d_history(2,:); u_d_history];
u_noise_history = [u_noise_history(2,:); u_noise_history];
u_fb_history = [u_fb_history(2,:); u_fb_history];

%% PLOTS

figure;
hold on
plot(t_history, x_history(:,1))
plot(t_history,x_history(:,2))
plot(t_history, x_d_history(:,1))
plot(t_history,x_d_history(:,2))
title('State Plotted Over Time')
legend('theta1', 'theta2', 'theta_d1', 'theta_d2');

figure;
hold on
plot(t_history, error(:,1))
plot(t_history,error(:,2))
plot(t_history, error(:,3), '--')
plot(t_history,error(:,4), '--')
title('State Error Plotted Over Time')
legend('theta1 error', 'theta2 error', 'theta1\_dot error', 'theta2\_dot error', 'Location', 'Best');

figure;
hold on
plot(t_history, u_noise_history(:,1))
plot(t_history, u_d_history(:,1))
plot(t_history, u_fb_history(:,1))
title('Controller input for Motor 1')
legend( 'u_1', 'u\_ff_1', 'u\_fb_1', 'Location', 'BestOutside');

figure;
hold on
plot(t_history, u_noise_history(:,2))
plot(t_history, u_d_history(:,2))
plot(t_history, u_fb_history(:,2))
title('Controller input for Motor 2')
legend('u_2', 'u\_ff_2', 'u\_fb_2',  'Location', 'BestOutside');


[videoFile, videoPath] = uiputfile('F:\*.*', 'Save Video As');
videoFig = figure;
xlim([-2, 2]);
ylim([-2, 2]);
xlabel('X Position relative to Robot Origin')
ylabel('Y Position relative to Robot Origin')
title('Real and Desired Robot State and Path Traced over Time')
axis square;
grid on;
myVideo = VideoWriter([videoPath, videoFile], 'MPEG-4'); %Initialize a videoWriter object
open(myVideo); %Open the videoWriter object


x_LineSpec = 'b-';
x_d_LineSpec = 'r:';
x_LineSpec2 = 'c-';
x_d_LineSpec2 = 'm:';



time_factor = 100; %decrease to increase FPS

for i = 1:size(x_history, 1)
    if time_factor*i > size(x_history, 1)
        break;
    end
    set(gca, 'nextplot', 'replacechildren') %Replace only the area within the plot axes
    

    plot_state(x_history, time_factor*i, x_LineSpec, x_LineSpec2, 'x', 'real')
    hold on
    plot_state(x_d_history, time_factor*i, x_d_LineSpec, x_d_LineSpec2, 'x_d', 'not real') 
    hold off
    
    title(['Real and Desired Robot State and Path Traced over Time', newline, 'Time = ',num2str(t_history(time_factor*i))]);
    legend('x', 'x', 'x end effector traced', 'x_d', 'x_d', 'x_d end effector traced', 'Location', 'BestOutside');
    %pause((t_history(end) - t_history(1))/length(t_history));    
    pause(0.00001);
    frame = getframe(videoFig); %Get plot image (frame) for the kth iteration 
    writeVideo(myVideo, frame); %Save frame to videoWriter object
end


close(myVideo); %Close the videoWriter object

function out = plot_state(state_history, ii, LineSpec, LineSpec2, DisplayName, real)
global l1 l2 l1_real l2_real LineWidth;
theta1 = state_history(ii,1);
theta2 = state_history(ii,2);

origin = [0,0];

if strcmp(real, 'real') == 1
    point1 = [l1_real*cos(theta1), l1_real*sin(theta1)];
    point2 = [l1_real*cos(theta1) + l2_real*cos(theta1+theta2), l1_real*sin(theta1) + l2_real*sin(theta1+theta2)];
    end_effector = path_trace(state_history, ii, 'real');
else 
    point1 = [l1*cos(theta1), l1*sin(theta1)];
    point2 = [l1*cos(theta1) + l2*cos(theta1+theta2), l1*sin(theta1) + l2*sin(theta1+theta2)];
    end_effector = path_trace(state_history, ii, 'not real');
end


plot([origin(1), point1(1)], [origin(2), point1(2)], LineSpec, ...
     [point1(1), point2(1)], [point1(2), point2(2)], LineSpec, ...
     'LineWidth', LineWidth, 'DisplayName', DisplayName);

hold on
plot(end_effector(:,1), end_effector(:,2), LineSpec2,'LineWidth', .9, 'DisplayName', [DisplayName, ' path traced'])

 
axis square

xlim([-2, 2]);
ylim([-2, 2]);
end

function out = path_trace(state_history,ii, real)
    global l1 l2 l1_real l2_real ;
    
    theta1 = state_history(1,1);
    theta2 = state_history(1,2);

    if strcmp(real, 'real')
        end_effector_path = [l1_real*cos(theta1) + l2_real*cos(theta1+theta2), l1_real*sin(theta1) + l2_real*sin(theta1+theta2)];
    
        for q = 2:ii

            theta1 = state_history(q,1);
            theta2 = state_history(q,2);

            end_effector_path = [end_effector_path;[l1_real*cos(theta1) + l2_real*cos(theta1+theta2), l1_real*sin(theta1) + l2_real*sin(theta1+theta2)]];
        end
    else 
        end_effector_path = [l1*cos(theta1) + l2*cos(theta1+theta2), l1*sin(theta1) + l2*sin(theta1+theta2)];

        for q = 2:ii

            theta1 = state_history(q,1);
            theta2 = state_history(q,2);

            end_effector_path = [end_effector_path;[l1*cos(theta1) + l2*cos(theta1+theta2), l1*sin(theta1) + l2*sin(theta1+theta2)]];
        end
    end
    out = end_effector_path;
end






%% EOM FOR THE ROBOTIC ARM SYSTEM
function dx = eom(t, x, start_pos, end_pos, T)
x = x.';
random_number = 0.01*randn(1,4);
x = x + random_number;
global x_prev x_d_history t_history x_history u_max u_d_history u_noise_history t_history_offset;
% GENERATE THE DESIRED STATE AND DESIRED INPUT
[x_d, u_d] = generate_desired_path(start_pos, end_pos, T, t, x_prev);

% GENERATE THE LINEARIZED ERROR DYNAMICS
[A, B] = error_dynamics(x_d, u_d);
% CALCULATE THE CONTROLLER
[~, u] = LQR_cntrl(A, B, x_d, u_d, x);
u_pi = position_PI_controller(x_d, x, t);

u = u+u_pi;
u_noise = u + 0.1*randn(1,2);
for i = 1:size(u_noise,1)
    for j = 1:2
        if abs(u_noise(i,j))>u_max
            if u_noise(i,j) < 0
                u_noise(i,j) = -u_max;
            elseif u_noise(i,j) > 0
                u_noise(i,j) = u_max;
            end
        end
    end
end
            


% CALCULATING THE STATE SPACE OUTPUT
theta_2dot = cal_theta_2dot(x, u_noise);
dx = [x(3);
      x(4);
      theta_2dot(1);
      theta_2dot(2)];

% STORE THE CURRENT x

x_prev = x;
x = x - random_number

x_d_history = [x_d_history; x_d];
x_history = [x_history; x];
t_history = [t_history; t_history_offset + t];

u_d_history = [u_d_history; u_d];
u_noise_history = [u_noise_history; u_noise];

end

%% PI POSITION ADJUSTING CONTROLLER
function u_pi = position_PI_controller(x_d, x, t)
global integral_error t_old
kp = 30;
ki = 0.5;
dt = t -t_old;
desired_coord = cart_coord([x_d(1), x_d(2)]);
current_coord = cart_coord_real([x(1), x(2)]);
err_vec = desired_coord - current_coord;

u_p = kp*desired_angular_vel(x , err_vec);
integral_error = integral_error+err_vec*dt;
u_i = ki*integral_error;

u_pi = u_p+u_i;

t_old = t;
end

%% CALCULATING THE END POINT COORDINATE
function end_pos = cart_coord_real(theta_ite)
global l1_real l2_real;
theta1 = theta_ite(1);
theta2 = theta_ite(2);
x = l1_real*cos(theta1)+l2_real*cos(theta1+theta2);
y = l1_real*sin(theta1)+l2_real*sin(theta1+theta2);
end_pos = [x, y];

end

%% FUNCTION FOR CALCULATING THETA_2DOT
function theta_2dot = cal_theta_2dot(x, u)
global l1_real l2_real m1_real m2_real g;
theta1 = x(1);
theta2 = x(2);
theta1_dot = x(3);
theta2_dot = x(4);

M = [m1_real*l1_real^2+m2_real*(l1_real^2+2*l1_real*l2_real*cos(theta2)+l2_real^2), m2_real*(l1_real*l2_real*cos(theta2)+l2_real^2);
    m2_real*(l1_real*l2_real*cos(theta2)+l2_real^2), m2_real*l2_real^2];

C = [-m2_real*l1_real*l2_real*sin(theta2)*(2*theta1_dot*theta2_dot+theta2_dot^2), m2_real*l1_real*l2_real*theta1_dot^2*sin(theta2)].';

G = [(m1_real+m2_real)*l1_real*g*cos(theta1)+m2_real*g*l2_real*cos(theta1+theta2), m2_real*g*l2_real*cos(theta1+theta2)].';

%M_inv = inv(M);

% CALCULATE THE THETA_2DOT
theta_2dot = M\(u.')-M\C-M\G;
theta_2dot = theta_2dot.';

end

%% LQR CONTROLLER DESIGN 
function [sys_err_cl, u] = LQR_cntrl(A, B, x_d, u_d, x)

%DESIGN Q MATRIX
Q_diag = Q_Diag(x, x_d);   % Design Q(x_d(t))
Q = diag(Q_diag);

%DESIGN R MATRIX
R_diag = R_Diag(x, x_d);    % Design R(x_d(t))
R = diag(R_diag);

%COMPUTE THE CONTROLLER PARAMETERS
[K, ~, ~] = lqr(A, B, Q, R);

u = u_d.' - K*((x - x_d).'); % u here is 2*1
u = u.'; % convert u to 1*2

%FORM THE CL ERROR STATESPACE SYSTEM 
sys_err_cl = ss(A-B*K, B, eye(4), zeros(4, 2));

%PLOT STEP RESPONSE
%step(sys_err_cl)
%xlabel('Time (sec)')
%ylabel('States')
end


function out = Q_Diag(x, x_d)

out = [100,100,10,10];

end

function out = R_Diag(x, x_d)

out = [1,1];

end


%% POLE PLACEMENT CONTROLLER DESIGN

function [sys_err_cl, u] = pole_placement(A, B, x_d, u_d, x)

eigenvalues = eig(A);
% for i = 1:length(eigenvalues)
%     if real(eigenvalues(i)) > 0
%         disp("Right half plane present");
% 
%     else
%         continue;
%     end 
% end

if rank(ctrb(A,B)) ~= 4
    disp("Not controllable");
end

%DESIGN DESIRED POLE LOCATION VECTOR
p = desired_poles(x_d);

%COMPUTE THE CONTROLLER PARAMETERS
K = place(A,B,p);

u = u_d.' - K*((x - x_d).'); % u here is 2*1
u = u.'; % convert u to 1*2

% Output for closed loop system
sys_err_cl = ss(A-B*K, B, eye(4), zeros(4, 2));

end


function p = desired_poles(x_d)

 p = [-1, -.5, -.1, -2];
 
end

%% FUNCTION FOR CALCULATING END POINT VELOCITY
function pos_dot = cal_end_vel(x_current)
global l1 l2;
theta1 = x_current(1);
theta2 = x_current(2);
theta_dot = [x_current(3), x_current(4)];
J = [-l1*sin(theta1)-l2*sin(theta1+theta2), -l2*sin(theta1+theta2);
    l1*cos(theta1)+l2*cos(theta1+theta2), l2*cos(theta1+theta2)];

pos_dot = J*(theta_dot.');
pos_dot = pos_dot.';
end

%% FUNCTION THAT GENERATE DESIRED PATH AND INPUT
function [x_d, u_d] = generate_desired_path(start_pos, end_pos, T, t, x_previous)
[x_d, y_d, x_dot_d, y_dot_d, x_2dot_d, y_2dot_d] = desired_pos(start_pos, end_pos, T, t);
%GENERATE THE DESIRED ANGULAR POSITIONS
pos_desired = [x_d, y_d];
angle_desired = cal_angle(pos_desired, x_previous);
%GENERATE THE DESIRED ANGULAR VELOCITIES
pos_dot = [x_dot_d, y_dot_d];
x_current = [angle_desired, 0, 0]; %0s here just work as place holders
theta_dot = desired_angular_vel(x_current , pos_dot);
%GENERATE DESIRED STATE
x_d = [angle_desired, theta_dot];
%GENERATE THE DESIRED INPUT TORQUE
pos_2dot = [x_2dot_d, y_2dot_d];
theta_2dot = desired_angular_acc(x_d, pos_dot, pos_2dot);
u_d = cal_torque(x_d, theta_2dot);

end

%% FUNCTION FOR CALCULATING DESIRED TORQUE INPUT
function u_d = cal_torque(x_current, theta_2dot)
global m1 m2 l1 l2 g;
theta1 = x_current(1);
theta2 = x_current(2);
theta1_dot = x_current(3);
theta2_dot = x_current(4);

M = [m1*l1^2+m2*(l1^2+2*l1*l2*cos(theta2)+l2^2), m2*(l1*l2*cos(theta2)+l2^2);
    m2*(l1*l2*cos(theta2)+l2^2), m2*l2^2];

C = [-m2*l1*l2*sin(theta2)*(2*theta1_dot*theta2_dot+theta2_dot^2), m2*l1*l2*theta1_dot^2*sin(theta2)].';

G = [(m1+m2)*l1*g*cos(theta1)+m2*g*l2*cos(theta1+theta2), m2*g*l2*cos(theta1+theta2)].';

u_d = (M*(theta_2dot.')+C+G).';

end
%% FUNCTION FOR CALCULATING DESIRED ANGLES
function angle_desired = cal_angle(pos_desired, x_current)
theta_ite = [x_current(1), x_current(2)];
 
%INITIATE DISTANCE ERROR
pos_error = pos_desired - cart_coord(theta_ite);

while (norm(pos_error)>0.00001)
   inv_jacobian = cal_inv_jacobian(theta_ite);
   del_angle = (inv_jacobian*pos_error.').'; 
   theta_ite = theta_ite + del_angle;
   pos_error = pos_desired - cart_coord(theta_ite);
   
end
angle_desired = theta_ite;

end

%% CALCULATING THE JACOBIAN
function inv_jacobian = cal_inv_jacobian(theta_ite)
global l1 l2;
theta1 = theta_ite(1);
theta2 = theta_ite(2);

J = [-l1*sin(theta1)-l2*sin(theta1+theta2), -l2*sin(theta1+theta2);
    l1*cos(theta1)+l2*cos(theta1+theta2), l2*cos(theta1+theta2)];

inv_jacobian = inv(J);

end

%% CALCULATING THE END POINT COORDINATE
function end_pos = cart_coord(theta_ite)
global l1 l2;
theta1 = theta_ite(1);
theta2 = theta_ite(2);
x = l1*cos(theta1)+l2*cos(theta1+theta2);
y = l1*sin(theta1)+l2*sin(theta1+theta2);
end_pos = [x, y];

end

%% FUNCTION THAT OUTPUT POSITIONT OF END POINT
function [x, y, x_dot, y_dot, x_2dot, y_2dot] = desired_pos(start_pos, end_pos, T, t)
a2 = 3/T^2;
a3 = -2/T^3;
s = a2*t^2+a3*t^3;
s_dot = 2*a2*t+3*a3*t^2;
s_2dot = 2*a2+6*a3*t;
dir_vec = end_pos - start_pos;
x = dir_vec(1)*s+start_pos(1);
y = dir_vec(2)*s+start_pos(2);
x_dot = dir_vec(1)*s_dot;
y_dot = dir_vec(2)*s_dot;
x_2dot = dir_vec(1)*s_2dot;
y_2dot = dir_vec(2)*s_2dot;

end

%% FUNCTION THAT CALCUALTE THE DESIRED ANGULAR ACCELERATION
function theta_2dot = desired_angular_acc(x_current, pos_dot, pos_2dot)
global l1 l2;

x_dot = pos_dot(1);
y_dot = pos_dot(2);
x_2dot = pos_2dot(1);
y_2dot = pos_2dot(2);
theta1 = x_current(1);
theta2 = x_current(2);
theta1_dot = x_current(3);
theta2_dot = x_current(4);

theta_2dot = [(cos(theta1 + 2*theta2)*y_2dot - sin(theta1 + 2*theta2)*x_2dot - cos(theta1)*y_2dot + sin(theta1)*x_2dot + cos(theta1)*theta1_dot*x_dot + 2*cos(theta1)*theta2_dot*x_dot + sin(theta1)*theta1_dot*y_dot + 2*sin(theta1)*theta2_dot*y_dot - cos(theta1 + 2*theta2)*theta1_dot*x_dot - sin(theta1 + 2*theta2)*theta1_dot*y_dot)/(l1*(cos(2*theta2) - 1)), (l2*cos(theta1)*y_2dot - l2*sin(theta1)*x_2dot + l2*cos(theta2)^2*sin(theta1)*x_2dot - l2*cos(theta1)*x_dot*theta1_dot - l2*cos(theta1)*x_dot*theta2_dot + l1*cos(theta1)*sin(theta2)*x_2dot - l2*sin(theta1)*y_dot*theta1_dot - l2*sin(theta1)*y_dot*theta2_dot + l1*sin(theta1)*sin(theta2)*y_2dot - l2*cos(theta1)*cos(theta2)^2*y_2dot - l1*sin(theta1)*sin(theta2)*x_dot*theta1_dot + l2*cos(theta2)*sin(theta1)*sin(theta2)*y_2dot + l2*cos(theta1)*cos(theta2)^2*x_dot*theta1_dot + l2*cos(theta2)^2*sin(theta1)*y_dot*theta1_dot - l1*cos(theta1)*cos(theta2)*x_dot*theta2_dot + l2*cos(theta1)*cos(theta2)*sin(theta2)*x_2dot + l1*cos(theta1)*sin(theta2)*y_dot*theta1_dot - l1*cos(theta2)*sin(theta1)*y_dot*theta2_dot + l2*cos(theta1)*cos(theta2)*sin(theta2)*y_dot*theta1_dot - l2*cos(theta2)*sin(theta1)*sin(theta2)*x_dot*theta1_dot)/(l1*l2*(cos(theta2)^2 - 1))];

end

%% FUNCTION THAT CALCULATE THE DESIRED ANGULAR VELOCITY
function theta_dot = desired_angular_vel(x_current , pos_dot)
global l1 l2;

theta1 = x_current(1);
theta2 = x_current(2);

J = [-l1*sin(theta1)-l2*sin(theta1+theta2), -l2*sin(theta1+theta2);
    l1*cos(theta1)+l2*cos(theta1+theta2), l2*cos(theta1+theta2)];

theta_dot = J\(pos_dot.'); %same as inv(J)*pos_d_dot
theta_dot = theta_dot.';
end

%% ERROR DYNAMICS FUNCTION THAT OUTPUT ERROR DYNAMICS GIVEN CERTAIN X VALUE (PARAMETERS COPIED FROM THE OUTPUT OF LINEARIZED ERROR DYNAMICS FILE)
function [A, B] = error_dynamics(x_d, u_d)
global m1 m2 l1 l2 g;

theta1 = x_d(1);
theta2 = x_d(2);
theta1dot = x_d(3);
theta2dot = x_d(4);
tau1 = u_d(1);
tau2 = u_d(2);


A = [                                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                           1,                                                                                                       0;
                                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                           0,                                                                                                       1;
                                                                                                                                                 (g*(2*m1*sin(theta1) + m2*sin(theta1) - m2*sin(theta1 + 2*theta2)))/(l1*(2*m1 + m2 - m2*cos(2*theta2))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           (2*m2*cos(theta2)*l2^2*theta1dot^2 + 4*m2*cos(theta2)*l2^2*theta1dot*theta2dot + 2*m2*cos(theta2)*l2^2*theta2dot^2 + 2*l1*m2*cos(2*theta2)*l2*theta1dot^2 - 2*g*m2*sin(theta1 + 2*theta2)*l2 + 2*tau2*sin(theta2))/(l1*l2*(2*m1 + m2 - m2*cos(2*theta2))) - (2*m2*sin(2*theta2)*(2*l2*tau1 - 2*l2*tau2 - 2*l1*tau2*cos(theta2) + g*l1*l2*m2*cos(theta1 + 2*theta2) + l1^2*l2*m2*theta1dot^2*sin(2*theta2) + 2*l1*l2^2*m2*theta1dot^2*sin(theta2) + 2*l1*l2^2*m2*theta2dot^2*sin(theta2) - 2*g*l1*l2*m1*cos(theta1) - g*l1*l2*m2*cos(theta1) + 4*l1*l2^2*m2*theta1dot*theta2dot*sin(theta2)))/(l1^2*l2*(2*m1 + m2 - m2*cos(2*theta2))^2),                                                                 (m2*(2*l2*theta1dot*sin(theta2) + 2*l2*theta2dot*sin(theta2) + 2*l1*theta1dot*cos(theta2)*sin(theta2)))/(l1*(- m2*cos(theta2)^2 + m1 + m2)),                              (2*l2*m2*sin(theta2)*(theta1dot + theta2dot))/(l1*(m2*sin(theta2)^2 + m1));
 (g*sin(theta1 + theta2)*(l1^2*m1 + l1^2*m2 + l2^2*m2 + 2*l1*l2*m2*cos(theta2)))/(l1^2*l2*(- m2*cos(theta2)^2 + m1 + m2)) - ((g*l1*sin(theta1)*(m1 + m2) + g*l2*m2*sin(theta1 + theta2))*(l2 + l1*cos(theta2)))/(l1^2*l2*(- m2*cos(theta2)^2 + m1 + m2)), -(l1^2*m1*tau2*sin(2*theta2) + l1^2*m2*tau2*sin(2*theta2) - l2^2*m2*tau1*sin(2*theta2) + l2^2*m2*tau2*sin(2*theta2) - 2*l1^2*l2^2*m2^2*theta1dot^2 - l1^2*l2^2*m2^2*theta2dot^2 - l1*l2*m1*tau1*sin(theta2) + 2*l1*l2*m1*tau2*sin(theta2) - 2*l1*l2*m2*tau1*sin(theta2) + 4*l1*l2*m2*tau2*sin(theta2) + l1^3*l2*m1^2*theta1dot^2*cos(theta2) - l1*l2^3*m2^2*theta1dot^2*cos(theta2) - l1^3*l2*m2^2*theta1dot^2*cos(theta2) - l1*l2^3*m2^2*theta2dot^2*cos(theta2) + l1*l2*m2*tau1*sin(theta2)^3 - 2*l1*l2*m2*tau2*sin(theta2)^3 + l1*l2^3*m2^2*theta1dot^2*cos(theta2)^3 + l1^3*l2*m2^2*theta1dot^2*cos(theta2)^3 + l1*l2^3*m2^2*theta2dot^2*cos(theta2)^3 - 2*l1^2*l2^2*m1*m2*theta1dot^2 - l1^2*l2^2*m1*m2*theta2dot^2 - 2*l1^2*l2^2*m2^2*theta1dot*theta2dot + g*l1*l2^2*m2^2*sin(theta1) + 2*l1^2*l2^2*m2^2*theta1dot^2*cos(theta2)^2 + l1^2*l2^2*m2^2*theta2dot^2*cos(theta2)^2 + l1*l2^3*m1*m2*theta1dot^2*cos(theta2) + l1*l2^3*m1*m2*theta2dot^2*cos(theta2) - 2*l1*l2^3*m2^2*theta1dot*theta2dot*cos(theta2) + l1^3*l2*m1*m2*theta1dot^2*cos(theta2)^3 + 2*l1*l2^3*m2^2*theta1dot*theta2dot*cos(theta2)^3 - g*l1^2*l2*m1^2*cos(theta2)*sin(theta1) + g*l1^2*l2*m2^2*cos(theta2)*sin(theta1) - 2*l1^2*l2^2*m1*m2*theta1dot*theta2dot + g*l1*l2^2*m1*m2*sin(theta1) + 4*l1^2*l2^2*m1*m2*theta1dot^2*cos(theta2)^2 + 2*l1^2*l2^2*m1*m2*theta2dot^2*cos(theta2)^2 + 2*l1^2*l2^2*m2^2*theta1dot*theta2dot*cos(theta2)^2 - g*l1*l2^2*m2^2*cos(theta2)^2*sin(theta1) - g*l1^2*l2*m2^2*cos(theta2)^3*sin(theta1) + 2*l1*l2^3*m1*m2*theta1dot*theta2dot*cos(theta2) + 4*l1^2*l2^2*m1*m2*theta1dot*theta2dot*cos(theta2)^2 - 2*g*l1*l2^2*m1*m2*cos(theta2)^2*sin(theta1) - g*l1^2*l2*m1*m2*cos(theta2)^3*sin(theta1))/(l1^2*l2^2*(- m2*cos(theta2)^2 + m1 + m2)^2), -(2*sin(theta2)*(l1^2*m1*theta1dot + l1^2*m2*theta1dot + l2^2*m2*theta1dot + l2^2*m2*theta2dot + 2*l1*l2*m2*theta1dot*cos(theta2) + l1*l2*m2*theta2dot*cos(theta2)))/(l1*l2*(- m2*cos(theta2)^2 + m1 + m2)), -(m2*sin(theta2)*(2*theta1dot + 2*theta2dot)*(l2 + l1*cos(theta2)))/(l1*(- m2*cos(theta2)^2 + m1 + m2))];
 
 
B = [                                                               0,                                                                                                    0;
                                                               0,                                                                                                    0;
                         2/(l1^2*(2*m1 + m2 - m2*cos(2*theta2))),                                      -(l2 + l1*cos(theta2))/(l1^2*l2*(- m2*cos(theta2)^2 + m1 + m2));
 -(l2 + l1*cos(theta2))/(l1^2*l2*(- m2*cos(theta2)^2 + m1 + m2)), (l1^2*m1 + l1^2*m2 + l2^2*m2 + 2*l1*l2*m2*cos(theta2))/(l1^2*l2^2*m2*(- m2*cos(theta2)^2 + m1 + m2))];
 
end