clear all;
close all;

syms theta1(t) theta2(t) l1 l2 x(t) y(t)

x_dot = diff(x, t);
y_dot = diff(y, t);

J = [-l1*sin(theta1)-l2*sin(theta1+theta2), -l2*sin(theta1+theta2);
    l1*cos(theta1)+l2*cos(theta1+theta2), l2*cos(theta1+theta2)];

J_inv = inv(J);

pos_dot = [x_dot, y_dot].';

theta_dot = J_inv*pos_dot;




theta_2dot = simplify(diff(theta_dot, t)) 

%% pos_2dot VALUE IS COPIED BELOW AND DIRIVATIVES ARE BEING REPLACED