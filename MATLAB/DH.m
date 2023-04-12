clc
clear all
close all

% HomogeneousTransform(alpha, a, theta, d)

%% #1

syms L1 L2 L3 theta1 theta2 theta3

T01 = HomogeneousTransform(0.0, 0.0, theta1, 0.0)
T12 = HomogeneousTransform(0.0, L1,  theta2, 0.0)
T23 = HomogeneousTransform(0.0, L2,  theta3, 0.0) 
T34 = HomogeneousTransform(0.0, L3,  0.0,    0.0) 

T02 = T01*T12
T03 = T01*T12*T23
T04 = simplify(T01*T12*T23*T34)

x = T04(1,4)
y = T04(2,4)
z = T04(3,4)

Jv = simplify([diff(x, theta1), diff(x, theta2), diff(x, theta3);
               diff(y, theta1), diff(y, theta2), diff(y, theta3);
               diff(z, theta1), diff(z, theta2), diff(z, theta3)])

J_wrt0 = [Jv([1:2],:);
          1 1 1]

J_wrt1 = simplify(transpose(T01([1:3],[1:3])) * J_wrt0)
J_wrt2 = simplify(transpose(T02([1:3],[1:3])) * J_wrt0)

simplify(det(J_wrt1))
simplify(det(J_wrt2))

J_wrt1 = subs(J_wrt1, theta2, 0)
J_wrt2 = subs(J_wrt2, theta2, 0)

rank(J_wrt2)