function [fitline_coefs,bestEndPoints] = ransac(r, theta)
% returns evenly spaced points along fitted lines using the RANSAC
% methodology

%eliminate zeros in dataset
index = find(r~=0 & r<3);
r_clean = r(index);
theta_clean = theta(index);

% convert points to cartesean
[x,y] = pol2cart(deg2rad(theta_clean), r_clean);

% offset lidar origin and pack into points object
x = x-.084;
points=[x,y];
points_raw=[x,y];

% set args for robustLineFit function
bestOutlierSet = points;
d = 0.005;
n = 1000;
visualize = 0;
nn = 1;

% fit lines to data until there's only a few points left
while size(bestOutlierSet, 1) > 5   
    [fitline_coefs(nn,:),bestInlierSet,bestOutlierSet,bestEndPoints(:,:,nn)]= robustLineFit(x,y,d,n,visualize);
    if isnan(fitline_coefs(nn,1))
        disp('All lines identified')
        break;
    end
    
    % set the points equal to previous values compounding
    x = bestOutlierSet(:,1);
    y = bestOutlierSet(:,2);
    theta_clean=rad2deg(theta_clean);
    
    nn=nn+1;
end

% plot everything
figure(1);
plot(points_raw(:,1),points_raw(:,2),'.');
hold on

for kk=1:size(bestEndPoints,3)
    plot(bestEndPoints(:,1,kk),bestEndPoints(:,2,kk),'r');
end

plot(bestInlierSet(:,1), bestInlierSet(:,2), 'ks')
plot(bestOutlierSet(:,1),bestOutlierSet(:,2),'bs')
title(['RANSAC with d=' num2str(d) ' and n=' num2str(n)])
xlabel('[m]')
ylabel('[m]')

end