clc;
clear all;
close all;

hz_ = 1000;
tick = 4000;
%% HW 8

fid = fopen('/home/kwan/kwan_robotics_assignment/graph/HW3.txt');
data = textscan(fid, '%f%f%f%f%f%f%f');
x_des = [data{1} data{2} data{3}];
x_1 = [data{4} data{5} data{6}];
v_1 = [data{7}];
fclose(fid)

%% x

figure('Position', [10 10 800 600])
hold on
plot([1:tick]/hz_, x_des(:,1),'--','Color','k','LineWidth', 5)
plot([1:tick]/hz_, x_1(:,1),'Color','r')
hold off
legend("X cubic command [m]", "HW 10-3 X pos [m]")
title('Cubic command')

%% Y

figure('Position', [10 10 800 600])
hold on
plot([1:tick]/hz_, x_des(:,2),'--','Color','k','LineWidth', 5)
plot([1:tick]/hz_, x_1(:,2),'Color','r')
ylim([-0.03 0.00])
hold off
legend("Y cubic command [m]", "HW 10-3 Y pos [m]")
title('Cubic command')

%% Z

figure('Position', [10 10 800 600])
hold on
plot([1:tick]/hz_, x_des(:,3),'--','Color','k','LineWidth', 5)
plot([1:tick]/hz_, x_1(:,3),'Color','r')
hold off
legend("Z cubic command [m]", "HW 10-3 Z pos [m]")
title('Cubic command')

%% vel

vmax = 0.3*ones(tick,1);
figure('Position', [10 10 800 600])
hold on
plot([1:tick]/hz_, vmax,'--','Color','k','LineWidth', 5)
plot([1:tick]/hz_, v_1(:,1),'Color','r')
hold off
legend("V max [m/s]", "HW 10-3 Velocity [m/s]")
title('Velocity saturation')

%% 3d plot
% [X,Y,Z] = sphere
% 
% r =0.1;
% X2 = X * r;
% Y2 = Y * r;
% Z2 = Z * r;
% 
% hold on
% surf(X2+0.15,Y2-0.012,Z2+0.65)
% plot3(x_1(:,1), x_1(:,2), x_1(:,3))
% 
% xlim([-0.5 0.5])
% ylim([-0.5 0.5])
% zlim([0 1])