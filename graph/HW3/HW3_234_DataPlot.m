clc;
clear all;
close all;

hz_ = 100;

%% HW 3-2

fid = fopen('/home/kwan/kwan_robotics_assignment/graph/HW3/HW3-2.txt');
data = textscan(fid, '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f');

rot_des = [data{1} data{2} data{3} data{4} data{5} data{6} data{7} data{8} data{9}]
x_32 = [data{10} data{11} data{12}]
xd_32 = [data{13} data{14} data{15} data{16} data{17} data{18}]
rot_32 = [data{19} data{20} data{21} data{22} data{23} data{24} data{25} data{26} data{27}]

fclose(fid)
%% HW 3-3
fid = fopen('/home/kwan/kwan_robotics_assignment/graph/HW3/HW3-3.txt');
data = textscan(fid, '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f');

x_des = [data{1} data{2} data{3}]
xd_des = [data{4} data{5} data{6} data{7} data{8} data{9}] 
x_33 = [data{10} data{11} data{12}]
xd_33 = [data{13} data{14} data{15} data{16} data{17} data{18}]
rot_33 = [data{19} data{20} data{21} data{22} data{23} data{24} data{25} data{26} data{27}]
fclose(fid)
    
%% HW 3-4

fid = fopen('/home/kwan/kwan_robotics_assignment/graph/HW3/HW3-4.txt');
data = textscan(fid, '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f');

x_34 = [data{1} data{2} data{3}]
xd_34 = [data{4} data{5} data{6} data{7} data{8} data{9}]
rot_34 = [data{10} data{11} data{12} data{13} data{14} data{15} data{16} data{17} data{18}]

fclose(fid)

%% x pos
set(groot,'defaultAxesXGrid','on')
set(groot,'defaultAxesYGrid','on')

figure('position', [10 10 800 600])
hold on
plot([1:400]/hz_, x_des(:,1),'--','Color','k','LineWidth', 5)
plot([1:400]/hz_, x_32(:,1),'Color','g')
plot([1:400]/hz_, x_33(:,1),'Color','b')
plot([1:400]/hz_, x_34(:,1),'Color','r')
hold off
legend("desired pos [m]", "HW 3-2 actual pos [m]", "HW 3-3 actual pos [m]", "HW 3-4 actual pos [m]")
title('X dir')
ylim([0.2 0.6])

figure('position', [10 10 800 600])
hold on
plot([1:400]/hz_, x_des(:,2), '--','Color','k','LineWidth', 5)
plot([1:400]/hz_, x_32(:,2),'Color','g')
plot([1:400]/hz_, x_33(:,2),'Color','b')
plot([1:400]/hz_, x_34(:,2),'Color','r')
hold off
legend("desired pos [m]", "HW 3-2 actual pos [m]", "HW 3-3 actual pos [m]", "HW 3-4 actual pos [m]")
title('Y dir')
ylim([-0.05 0.4])

figure('position', [10 10 800 600])
hold on
plot([1:400]/hz_, x_des(:,3), '--','Color','k','LineWidth', 5)
plot([1:400]/hz_, x_32(:,3),'Color','g')
plot([1:400]/hz_, x_33(:,3),'Color','b')
plot([1:400]/hz_, x_34(:,3),'Color','r')
legend("desired pos [m]", "HW 3-2 actual pos [m]", "HW 3-3 actual pos [m]", "HW 3-4 actual pos [m]")
hold off
title('Z dir')
ylim([0.645 0.655])

%% x vel

figure()
subplot(6,1,1)
hold on
plot([1:400]/hz_, xd_des(:,1), '--','Color','k','LineWidth', 5)
plot([1:400]/hz_, xd_32(:,1),'Color','g')
plot([1:400]/hz_, xd_33(:,1),'Color','b')
plot([1:400]/hz_, xd_34(:,1),'Color','r')
legend("desired linvel [m/s]", "HW 3-2 actual linvel [m/s]", "HW 3-3 actual linvel [m/s]", "HW 3-4 actual linvel [m/s]")
hold off
title('X dir')

subplot(6,1,2)
hold on
plot([1:400]/hz_, xd_des(:,2), '--','Color','k','LineWidth', 5)
plot([1:400]/hz_, xd_32(:,2),'Color','g')
plot([1:400]/hz_, xd_33(:,2),'Color','b')
plot([1:400]/hz_, xd_34(:,2),'Color','r')
hold off
title('Y dir')

subplot(6,1,3)
hold on
plot([1:400]/hz_, xd_des(:,3), '--','Color','k','LineWidth', 5)
plot([1:400]/hz_, xd_32(:,3),'Color','g')
plot([1:400]/hz_, xd_33(:,3),'Color','b')
plot([1:400]/hz_, xd_34(:,3),'Color','r')
hold off
title('Z dir')

subplot(6,1,4)
hold on
plot([1:400]/hz_, xd_des(:,4), '--','Color','k','LineWidth', 5)
plot([1:400]/hz_, xd_32(:,4),'Color','g')
plot([1:400]/hz_, xd_33(:,4),'Color','b')
plot([1:400]/hz_, xd_34(:,4),'Color','r')
legend("desired angvel [rad/s]", "HW 3-2 actual angvel [rad/s]", "HW 3-3 actual angvel [rad/s]", "HW 3-4 actual angvel [rad/s]")
hold off
title('X dir')

subplot(6,1,5)
hold on
plot([1:400]/hz_, xd_des(:,5), '--','Color','k','LineWidth', 5)
plot([1:400]/hz_, xd_32(:,5),'Color','g')
plot([1:400]/hz_, xd_33(:,5),'Color','b')
plot([1:400]/hz_, xd_34(:,5),'Color','r')
hold off
title('Y dir')

subplot(6,1,6)
hold on
plot([1:400]/hz_, xd_des(:,6), '--','Color','k','LineWidth', 5)
plot([1:400]/hz_, xd_32(:,6),'Color','g')
plot([1:400]/hz_, xd_33(:,6),'Color','b')
plot([1:400]/hz_, xd_34(:,6),'Color','r')
hold off
title('Z dir')

%% Rot

figure('position', [10 10 800 600])
for i = 1:9
    subplot(3,3,i)
    hold on
    plot([1:400]/hz_, rot_des(:,i), '--','Color','k','LineWidth', 5)
    plot([1:400]/hz_, rot_32(:,i),'Color','g')
    plot([1:400]/hz_, rot_33(:,i),'Color','b')   
    plot([1:400]/hz_, rot_34(:,i),'Color','r')

    hold off
end
    legend("desired rot", "HW 3-2 rot", "HW 3-3 rot", "HW 3-4 rot")
