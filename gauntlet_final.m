% Main class for navigating to the BoB. Aggregates breakout functions into
% a single executable file which takes no arguments and outputs plots and
% control to a connected neato simulation.

% initialize a connection to the simulation environment
rosshutdown(); rosinit('localhost',11311, 'NodeHost','host.docker.internal')

% clear and load
clear; clf;
load('BoB clean scan.mat');

% run RANSAC algorith on data
[fitline_coefs,bestEndPoints] = ransac(r, theta);

% set num points to interpolate for each line
line_num_pts = 10;
py_i = [];
px_i = [];
points = [];

% interpolate points given equations of found lines in space
for n = 1:size(fitline_coefs,1)
    x_range = bestEndPoints(:,1,n);
    px_i = [px_i; linspace(x_range(1),x_range(2),line_num_pts)];
    py_i = [py_i; fitline_coefs(n,1).*px_i(n,:)+fitline_coefs(n,2)];
    plot(px_i,py_i,'s','Color','k');
end

% calculate a gradient ascent path
delta = .9;
s0 = .3;
generate_ascent(px_i,py_i,delta, s0);
legend('Lidar Points','Found lines');

% drive along that path and collect data
drive_curve;

% plot that data on top of the existing path
plotData('bobrun.mat');
