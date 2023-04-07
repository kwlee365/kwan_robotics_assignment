clc
clear all
close all

% HomogeneousTransform(alpha, a, theta, d)

%% #1

syms L1 L2 theta1 theta2

% (a), (b)
digits(2)
T01 = HomogeneousTransform(0.0,       0.0, theta1, 0.0)
T12 = HomogeneousTransform(sym(pi/2), L1,  theta2, 0.0)
T23 = HomogeneousTransform(0.0,       L2,  0.0,    0.0) 

T02 = T01*T12;
T03 = simplify(T01*T12*T23)

% (c)

Jv = simplify([diff(T03(1,4), theta1) diff(T03(1,4), theta2);
               diff(T03(2,4), theta1) diff(T03(2,4), theta2);
               diff(T03(3,4), theta1) diff(T03(3,4), theta2)])

Jw = [T01([1:3],3), T02([1:3],3)]

Jo = [Jv;Jw];

% (d)

T03 = subs(T03, theta1, 0.0);
T03 = subs(T03, theta2, sym(pi/2))

Jo = subs(Jo, theta1, 0.0);
Jo = subs(Jo, theta2, pi/2)


e4 = 0.5 * (1 + T03(1,1) + T03(2,2) + T03(3,3));
e1 = (T03(3,2) - T03(2,3))/(4*e4);
e2 = (T03(1,3) - T03(3,1))/(4*e4);
e3 = (T03(2,1) - T03(1,2))/(4*e4);

Er = (1/2) * [-e1 -e2 -e3;
               e4  e3 -e2;
              -e3  e4  e1;
               e2 -e1  e4]

J = [eye(3) zeros(3,3); zeros(4,3) Er] * Jo

%% #2 
clc
clear all
close all

syms L1 theta1 theta2 d3 Ltip

T01 = HomogeneousTransform(0.0,       0.0, theta1, 0.0)
T12 = HomogeneousTransform(sym(pi/2), L1,  theta2, 0.0)
T23 = HomogeneousTransform(sym(pi/2), 0.0, 0.0,    d3)
T34 = HomogeneousTransform(0.0,       0.0, 0.0,    0.0)

T02 = T01*T12
T03 = T01*T12*T23
T04 = T01*T12*T23*T34

x = T04(1,4);
y = T04(2,4);
z = T04(3,4);

Jv = simplify([diff(x, theta1), diff(x, theta2), diff(x, d3);
               diff(y, theta1), diff(y, theta2), diff(y, d3);
               diff(z, theta1), diff(z, theta2), diff(z, d3)])

Jw = [T01([1:3],3), T02([1:3],3), zeros(3,1)]

%% #3

clc; clear all; close all;

syms theta1 theta2 theta3 L1 L2 L3 x y

T01 = HomogeneousTransform(0.0,       0.0, theta1, 0.0);
T12 = HomogeneousTransform(sym(pi/2), 0.0, theta2, 0.0);
T23 = HomogeneousTransform(0.0,       L1,  theta3, 0.0);
T34 = HomogeneousTransform(0.0,       L2,  0.0,    L3);

T04 = simplify(T01*T12*T23*T34)

X = collect(T04(1,4), [cos(theta1) sin(theta1)])
Y = collect(T04(2,4), [cos(theta1) sin(theta1)])

A = [(L2*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) + L1*cos(theta2)), L3;
     -L3,                                                                      (L2*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) + L1*cos(theta2))]

TH = collect(simplify(inv(A)*[x;y]), [x y])
TH = simplify(subs(TH, y, 0))
TH = simplify(subs(TH, L3, 0)) 

T04 = subs(T04, L3, 0)
T04 = subs(T04, sin(theta1), 0)

x_ee = T04(1,4);
z_ee = T04(3,4);

simplify(subs(x_ee^2 + z_ee^2, cos(theta1),1))
%% #4

clc; clear all; close all;

syms d1 theta2 theta3 L2 L3 d4

T01 = HomogeneousTransform(0, 0,  0,      d1);
T12 = HomogeneousTransform(0, 0,  theta2, 0);
T23 = HomogeneousTransform(0, L2, theta3, 0);
T34 = HomogeneousTransform(0, L3, 0,      d4);

%f
T04 = simplify(T01*T12*T23*T34)

x = T04(1,4)
y = T04(2,4)
z = T04(3,4)

%g
Jv = simplify([diff(x, d1), diff(x, theta2), diff(x, theta3), diff(x, d4);
               diff(y, d1), diff(y, theta2), diff(y, theta3), diff(y, d4);
               diff(z, d1), diff(z, theta2), diff(z, theta3), diff(z, d4)]);

T02 = T01*T12;
T03 = T01*T12*T23;

Jw = [zeros(3,1), T02([1:3],3), T03([1:3],3), zeros(3,1)];

%h
r1 = T04([1:3],1);
r2 = T04([1:3],2);
r3 = T04([1:3],3);

Jr = [diff(r1, d1), diff(r1, theta2), diff(r1,  theta3), diff(r1, d4);
      diff(r2, d1), diff(r2, theta2), diff(r2,  theta3), diff(r2, d4);
      diff(r3, d1), diff(r3, theta2), diff(r3,  theta3), diff(r3, d4)];

%i
Er = [-CrossProductOpertator(r1);
      -CrossProductOpertator(r2);
      -CrossProductOpertator(r3)];

%% 5 

clc; clear all; close all;

syms theta1 theta2 theta3 L1 L2

T01 = HomogeneousTransform(0,         0,  theta1, 0);
T12 = HomogeneousTransform(sym(pi/2), 0,  theta2, 0);
T23 = HomogeneousTransform(0,         L1, theta3, 0);
T34 = HomogeneousTransform(0,         L2, 0,      0);

T04 = simplify(T01*T12*T23*T34)

x = T04(1,4);
y = T04(2,4);
z = T04(3,4);

Jv = simplify([diff(x, theta1), diff(x, theta2), diff(x,  theta3);
               diff(y, theta1), diff(y, theta2), diff(y,  theta3);
               diff(z, theta1), diff(z, theta2), diff(z,  theta3)]);

Jv = subs(Jv, L1, 1);
Jv = subs(Jv, L2, 1)

T02 = T01*T12;
R02 = T02([1:3],[1:3])

Jv_wrt_2ndframe = simplify(transpose(R02)*Jv)
Jv_wrt_2ndframe = subs(Jv_wrt_2ndframe, theta2, sym(pi/2))
A = subs(Jv_wrt_2ndframe, theta3, sym(0))

ATA = transpose(A)*A
AAT = A*transpose(A)
[V,D] = eig(ATA)
[U,D] = eig(AAT)

a = sqrt(5)

[0 0 1; 1 0 0; 0 1 0] * [a 0 0;0 0 0;0 0 0] * transpose([0 0 1; 2/a -1/a 0;1/a 2/a 0])
Ainv = [0 0 1; 2/a -1/a 0;1/a 2/a 0] * [1/a 0 0;0 0 0;0 0 0] * transpose([0 0 1; 1 0 0; 0 1 0])

NullspaceProjection = eye(3) - A*Ainv
NullspaceProjection = eye(3) - Ainv*A