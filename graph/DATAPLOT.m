clc;
clear all;
close all;

hz_ = 1000;
tick = 4000;
%% HW 8

fid = fopen('/home/kwan/kwan_robotics_assignment/graph/HW1-1.txt');
data = textscan(fid, '%f%f%f%f%f%f');
x_des_step = [data{1} data{2} data{3}];
x_1 = [data{4} data{5} data{6}];
fclose(fid)

fid = fopen('/home/kwan/kwan_robotics_assignment/graph/HW1-2.txt');
data = textscan(fid, '%f%f%f%f%f%f');
x_des_cubic = [data{1} data{2} data{3}];
x_2 = [data{4} data{5} data{6}];
fclose(fid)

fid = fopen('/home/kwan/kwan_robotics_assignment/graph/HW2.txt');
data = textscan(fid, '%f%f%f%f%f%f%f');
x_des2 = [data{1} data{2} data{3}];
x_3 = [data{4} data{5} data{6}];
v_sat = [data{7}];
fclose(fid)

fid = fopen('/home/kwan/kwan_robotics_assignment/graph/HW3.txt');
data = textscan(fid, '%f%f%f%f%f%f%f');
x_4 = [data{4} data{5} data{6}];
v_obs = [data{7}];
fclose(fid)


%% x

figure('Position', [10 10 800 600])
hold on
plot([1:tick]/hz_, x_des_step(:,1),'--','Color','k','LineWidth', 5)
plot([1:tick]/hz_, x_1(:,1),'Color','r')
ylim([0.56 0.57])
hold off
legend("X step command [m]", "HW 10-1-1 X pos [m]")
title('Step command')

figure('Position', [10 10 800 600])
hold on
plot([1:tick]/hz_, x_des_cubic(:,1),'--','Color','k','LineWidth', 5)
plot([1:tick]/hz_, x_2(:,1),'Color','r')
ylim([0.56 0.57])
hold off
legend("X cubic command [m]", "HW 10-1-2 X pos [m]")
title('Cubic command')

%% Y
figure('Position', [10 10 800 600])
hold on
plot([1:tick]/hz_, x_des_step(:,2),'--','Color','k','LineWidth', 5)
plot([1:tick]/hz_, x_1(:,2),'Color','r')
hold off
legend("Y step command [m]", "HW 10-1-1 Y pos [m]")
title('Step command')

figure('Position', [10 10 800 600])
hold on
plot([1:tick]/hz_, x_des_cubic(:,2),'--','Color','k','LineWidth', 5)
plot([1:tick]/hz_, x_2(:,2),'Color','r')
hold off
legend("Y cubic command [m]", "HW 10-1-2 Y pos [m]")
title('Cubic command')
%% Z

figure('Position', [10 10 800 600])
hold on
plot([1:tick]/hz_, x_des_step(:,3),'--','Color','k','LineWidth', 5)
plot([1:tick]/hz_, x_1(:,3),'Color','r')
ylim([0.65 0.66])
hold off
legend("Z step command [m]", "HW 10-1-1 Z pos [m]")
title('Step command')

figure('Position', [10 10 800 600])
hold on
plot([1:tick]/hz_, x_des_cubic(:,3),'--','Color','k','LineWidth', 5)
plot([1:tick]/hz_, x_2(:,3),'Color','r')
ylim([0.65 0.66])
hold off
legend("Z cubic command [m]", "HW 10-1-2 Z pos [m]")
title('Cubic command')