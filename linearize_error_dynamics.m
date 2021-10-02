clear all;
close all;

syms theta1 theta2 l1 l2 m1 m2 theta1dot theta2dot g tau1 tau2
%% EQUATION OF MOTIONS
M = [m1*l1^2+m2*(l1^2+2*l1*l2*cos(theta2)+l2^2), m2*(l1*l2*cos(theta2)+l2^2);
    m2*(l1*l2*cos(theta2)+l2^2), m2*l2^2];

C = [-m2*l1*l2*sin(theta2)*(2*theta1dot*theta2dot+theta2dot^2), m2*l1*l2*theta1dot^2*sin(theta2)].';

G = [(m1+m2)*l1*g*cos(theta1)+m2*g*l2*cos(theta1+theta2), m2*g*l2*cos(theta1+theta2)].';

tau = [tau1, tau2].';

%% MASS MATRIX INVERSE
M_inv = simplify(inv(M));

%% FUNCTION F FOR X_DOT = f(X,U)
f_up = [theta1dot, theta2dot].';
f_down = simplify(-M_inv*C-M_inv*G+M_inv*tau);
f = vertcat(f_up, f_down);

%% FINDING THE LINEARIZED A MATRIX
A_col1 = simplify(diff(f,theta1));
A_col2 = simplify(diff(f,theta2));
A_col3 = simplify(diff(f,theta1dot));
A_col4 = simplify(diff(f,theta2dot));
A = simplify(horzcat(A_col1, A_col2, A_col3, A_col4))

%% FINDING THE LINEARIZED B MATRIX
B_col1 = simplify(diff(f, tau1));
B_col2 = simplify(diff(f, tau2));
B = simplify(horzcat(B_col1, B_col2))