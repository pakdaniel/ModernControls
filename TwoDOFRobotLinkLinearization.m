syms q1 q2 qdot1 qdot2;
syms m1 m2 L1 L2 g;
syms tao1 tao2;

M = [m1*L1^2+m2*(L1^2+2*L1*L2*cos(q2)+L2^2),  m2*(L1*L2*cos(q2)+L2^2);
     m2*(L1*L2*cos(q2) + L2^2)             ,   m2*L2^2                ];
c = [-m2*L1*L2*sin(q2)*(2*qdot1*qdot2 + qdot2^2);
     m2*L1*L2*qdot1^2*sin(q2) ];
g = [(m1+m2)*L1*g*cos(q1) + m2*g*L2*cos(q1+q2) ;
      m2*g*L2*cos(q1+q2)                        ];

T = [tao1; 
     tao2];
 
qdotdot = -inv(M)*(c+g) + inv(M)*T

simplify(qdotdot)
simplify(qdotdot)