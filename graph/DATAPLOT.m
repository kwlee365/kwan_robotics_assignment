clc;
clear all;
close all;

hz_ = 1000;
tick = 4000;
%% HW 7

fid = fopen('/home/kwan/kwan_robotics_assignment/graph/HW1.txt');
data = textscan(fid, '%f%f');
q_des_step = [data{1}];
q4_1 = [data{2}];
fclose(fid)

fid = fopen('/home/kwan/kwan_robotics_assignment/graph/HW2.txt');
data = textscan(fid, '%f%f');
q4_2 = [data{2}];
fclose(fid)

fid = fopen('/home/kwan/kwan_robotics_assignment/graph/HW3.txt');
data = textscan(fid, '%f%f');
q_des_cubic = [data{1}];
q4_3 = [data{2}];
fclose(fid)

fid = fopen('/home/kwan/kwan_robotics_assignment/graph/HW4.txt');
data = textscan(fid, '%f%f');
q4_4 = [data{2}];
fclose(fid)

fid = fopen('/home/kwan/kwan_robotics_assignment/graph/HW5.txt');
data = textscan(fid, '%f%f');
q4_5 = [data{2}];
fclose(fid)

q_des_step = rad2deg(q_des_step);
q_des_cubic = rad2deg(q_des_cubic);
q4_1 = rad2deg(q4_1);
q4_2 = rad2deg(q4_2);
q4_3 = rad2deg(q4_3);
q4_4 = rad2deg(q4_4);
q4_5 = rad2deg(q4_5);

%% q4

figure('Position', [10 10 800 600])
hold on
plot([1:tick]/hz_, q_des_step(:,1),'--','Color','k','LineWidth', 5)
plot([1:tick]/hz_, q4_1(:,1),'Color','r')
plot([1:tick]/hz_, q4_2(:,1),'Color','b')
plot([1:tick]/hz_, q4_4(:,1),'Color','g')
hold off
legend("Joint 4 step command [rad]", "HW 7-1 joint pos [rad]", "HW 7-2 joint pos [rad]", "HW 7-4 joint pos [rad]")
title('Step command')

figure('Position', [10 10 800 600])
hold on
plot([1:tick]/hz_, q_des_cubic(:,1),'--','Color','k','LineWidth', 5)
plot([1:tick]/hz_, q4_3(:,1),'Color','r')
plot([1:tick]/hz_, q4_5(:,1),'Color','b')
hold off
legend("Joint 4 cubic command [rad]", "HW 7-3 joint pos [rad]", "HW 7-5 joint pos [rad]")
title('Cubic command')