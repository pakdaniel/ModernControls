clear all;
close all;

syms theta1 theta2 theta1_dot theta2_dot x_2dot y_2dot l1 l2

  

pos_2dot = [(cos(theta1 + 2*theta2)*y_2dot - sin(theta1 + 2*theta2)*x_2dot - cos(theta1)*y_2dot + sin(theta1)*x_2dot + cos(theta1)*theta1_dot*x_dot + 2*cos(theta1)*theta2_dot*x_dot + sin(theta1)*theta1_dot*y_dot + 2*sin(theta1)*theta2_dot*y_dot - cos(theta1 + 2*theta2)*theta1_dot*x_dot - sin(theta1 + 2*theta2)*theta1_dot*y_dot)/(l1*(cos(2*theta2) - 1)), (l2*cos(theta1)*y_2dot - l2*sin(theta1)*x_2dot + l2*cos(theta2)^2*sin(theta1)*x_2dot - l2*cos(theta1)*x_dot*theta1_dot - l2*cos(theta1)*x_dot*theta2_dot + l1*cos(theta1)*sin(theta2)*x_2dot - l2*sin(theta1)*y_dot*theta1_dot - l2*sin(theta1)*y_dot*theta2_dot + l1*sin(theta1)*sin(theta2)*y_2dot - l2*cos(theta1)*cos(theta2)^2*y_2dot - l1*sin(theta1)*sin(theta2)*x_dot*theta1_dot + l2*cos(theta2)*sin(theta1)*sin(theta2)*y_2dot + l2*cos(theta1)*cos(theta2)^2*x_dot*theta1_dot + l2*cos(theta2)^2*sin(theta1)*y_dot*theta1_dot - l1*cos(theta1)*cos(theta2)*x_dot*theta2_dot + l2*cos(theta1)*cos(theta2)*sin(theta2)*x_2dot + l1*cos(theta1)*sin(theta2)*y_dot*theta1_dot - l1*cos(theta2)*sin(theta1)*y_dot*theta2_dot + l2*cos(theta1)*cos(theta2)*sin(theta2)*y_dot*theta1_dot - l2*cos(theta2)*sin(theta1)*sin(theta2)*x_dot*theta1_dot)/(l1*l2*(cos(theta2)^2 - 1))];
pos_2dot = simplify(pos_2dot)