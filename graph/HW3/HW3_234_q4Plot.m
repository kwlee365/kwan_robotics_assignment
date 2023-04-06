clc;
clear all;
close all;

hz_ = 100;

%% HW 3-2

fid = fopen('/home/kwan/kwan_robotics_assignment/graph/HW3/HW3-2.txt');
data = textscan(fid, '%f%f');

qdes = data{1}
q_32 = data{2}

fclose(fid)
%% HW 3-3

fid = fopen('/home/kwan/kwan_robotics_assignment/graph/HW3/HW3-3.txt');
data = textscan(fid, '%f');

q_33 = data{1}

fclose(fid)
    
%% HW 3-4

fid = fopen('/home/kwan/kwan_robotics_assignment/graph/HW3/HW3-4.txt');
data = textscan(fid, '%f');

q_34 = data{1}

fclose(fid)

%% x pos
set(groot,'defaultAxesXGrid','on')
set(groot,'defaultAxesYGrid','on')

figure()
hold on
plot([1:400]/hz_, qdes(:,1),'--','Color','k','LineWidth', 5)
plot([1:400]/hz_, q_32(:,1),'Color','g')
plot([1:400]/hz_, q_33(:,1),'Color','b')
plot([1:400]/hz_, q_34(:,1),'Color','r')
hold off
legend("desired qpos [rad]", "HW 3-2 qpos [rad]", "HW 3-3 qpos [rad]", "HW 3-4 qpos [rad]")
title('Joint 4')