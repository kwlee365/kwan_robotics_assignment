clc;
clear all;
close all;

hz_ = 100;
tick = 700;
%% HW 6
fid = fopen('/home/kwan/kwan_robotics_assignment/graph/HW1.txt');
data = textscan(fid, '%f%f%f%f%f%f%f%f%f%f%f%f%f');
x_des = [data{1} data{2} data{3} data{4} data{5} data{6}]
x_ee = [data{7} data{8} data{9}]
x_4   = [data{10} data{11} data{12}]
h2    = [data{13}]
fclose(fid)

fid = fopen('/home/kwan/kwan_robotics_assignment/graph/HW2.txt');
data = textscan(fid, '%f%f%f%f%f%f%f%f%f%f%f%f%f');
x_des_2 = [data{1} data{2} data{3} data{4} data{5} data{6}]
x_ee_2 = [data{7} data{8} data{9}]
x_4_2   = [data{10} data{11} data{12}]
h2_2    = [data{13}]
fclose(fid)

%% EE

figure('position', [10 10 800 600])
hold on
plot([1:tick]/hz_, x_des(:,1),'--','Color','k','LineWidth', 5)
plot([1:tick]/hz_, x_ee(:,1),'Color','r')
plot([1:tick]/hz_, x_ee_2(:,1),'Color','b')
grid on
hold off
legend("Desired ee pos [m]", "HW 6-1 Actual ee pos [m]", "HW 6-2 Actual ee pos [m]")
title('X dir')

figure('position', [10 10 800 600])
hold on
plot([1:tick]/hz_, x_des(:,2),'--','Color','k','LineWidth', 5)
plot([1:tick]/hz_, x_ee(:,2),'Color','r')
plot([1:tick]/hz_, x_ee_2(:,2),'Color','b')
grid on
hold off
legend("Desired ee pos [m]", "HW 6-1 Actual ee pos [m]", "HW 6-2 Actual ee pos [m]")
title('Y dir')

figure('position', [10 10 800 600])
hold on
plot([1:tick]/hz_, x_des(:,3),'--','Color','k','LineWidth', 5)
plot([1:tick]/hz_, x_ee(:,3),'Color','r')
plot([1:tick]/hz_, x_ee_2(:,3),'Color','b')
grid on
hold off
legend("Desired ee pos [m]", "HW 6-1 Actual ee pos [m]", "HW 6-2 Actual ee pos [m]")
title('Z dir')

%% Link 4

figure('position', [10 10 800 600])
hold on
plot([1:tick]/hz_, x_des(:,4),'--','Color','k','LineWidth', 5)
plot([1:tick]/hz_, x_4(:,1),'Color','r')
plot([1:tick]/hz_, x_4_2(:,1),'Color','b')
grid on
hold off
legend("Desired link 4 pos [m]", "HW 6-1 Actual link 4 pos [m]", "HW 6-2 Actual link 4 pos [m]")
title('X dir')

figure('position', [10 10 800 600])
hold on
plot([1:tick]/hz_, x_des(:,5),'--','Color','k','LineWidth', 5)
plot([1:tick]/hz_, x_4(:,2),'Color','r')
plot([1:tick]/hz_, x_4_2(:,2),'Color','b')
grid on
hold off
legend("Desired link 4 pos [m]", "HW 6-1 Actual link 4 pos [m]", "HW 6-2 Actual link 4 pos [m]")
title('Y dir')

figure('position', [10 10 800 600])
hold on
plot([1:tick]/hz_, x_des(:,6),'--','Color','k','LineWidth', 5)
plot([1:tick]/hz_, x_4(:,3),'Color','r')
plot([1:tick]/hz_, x_4_2(:,3),'Color','b')
grid on
hold off
legend("Desired link 4 pos [m]", "HW 6-1 Actual link 4 pos [m]", "HW 6-2 Actual link 4 pos [m]")
title('Z dir')

%% h2

figure()
hold on
plot([1:tick]/hz_, h2(:,1),'Color','r')
plot([1:tick]/hz_, h2_2(:,1),'Color','b')
legend("HW 6-1 h2", "HW 6-2 h2")
grid on
hold off
