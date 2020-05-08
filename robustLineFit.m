function [fitline_coefs,bestInlierSet,bestOutlierSet,bestEndPoints]= robustLineFit(x,y,d,n,visualize)
%The [fitline_coefs,bestInlierSet,bestOutlierSet,bestEndPoints]= robustLineFit(r,theta,d,n) 
%function runs the RANSAC algorithm for n candidate lines and a threshold of d. The inputs r and
%theta are polar coordinates. The output fitline_coefs are the coefficients
%of the best fit line in the format [m b] where y=m*x+b. If you want
%to visualize, set visualize flag to 1, off is 0. Default is true.

 if ~exist('visualize','var')
     % visualize parameter does not exist, so default it to 1
      visualize = 1;
 end


%convert to Cartesian and plot again for verification
points=[x,y];

%now let's actually implement the RANSAC algorithm
 bestcandidates = [];
 bestInlierSet = zeros(0,2);
 bestOutlierSet = zeros(0,2);
 bestEndPoints = zeros(0,2);
for k=1:n %number of candidate lines to try
    
    
    %select two points at random using the 'datasample' function to define
    %the endpoints of the first candidate fit line
    candidates = datasample(points, 2, 'Replace', false);
    
    %Find the vector that points from point 2 to point 1
    v=(candidates(1,:)-candidates(2,:))';
    
    %Check the length of the vector v. If it is zero, the datasample
    %function chose the same point twice, and we need to resample. The
    %continue command will pass to the next iteration of the for loop.
    if norm(v) == 0
        continue;
    end
    
    %Determine whether points are outliers, we need to know the
    %perpendicular distance away from the candidate fit line. To do this,
    %we first need to define the perpendicular, or orthogonal, direction.
    orthv= [-v(2); v(1)];
    orthv_unit=orthv/norm(orthv); %make this a unit vector
    
    %Here, we are finding the distance of each scan point from one of the
    %endpoints of our candidate line. At this point this is not the
    %distance perpendicular to the candidate line.
    diffs = points - candidates(2,:);
    
    %Next, we need to project the difference vectors above onto the
    %perpendicular direction in 'orthv_unit'. This will give us the
    %orthogonal distances from the canidate fit line.
    orthdists=diffs*orthv_unit;
    
    %To identify inliers, we will look for points at a perpendicular
    %distance from the candidate fit line less than the threshold value.
    %The output will be a logic array, with a 1 if the statement is true
    %and 0 if false.
    inliers=abs(orthdists) < d;
    
    %we also want to check that there are no big gaps in our walls. To do
    %this, we are first taking the distance of each inlier away from an
    %endpoint (diffs) and projecting onto the best fit direction. We then
    %sort these from smallest to largest and take difference to find the
    %spacing between adjacent points. We then identify the maximum gap.
    biggestGap = max(diff(sort(diffs(inliers,:)*v/norm(v))));
    
    %Now, we check if the number of inliers is greater than the best we
    %have found. If so, the candidate line is our new best candidate. We
    %also make sure there are no big gaps.
    if biggestGap < 0.2  && sum(inliers) > size(bestInlierSet,1)
%          if sum(inliers) > size(bestInlierSet,1)
        bestInlierSet=points(inliers,:); %points where logical array is true
        bestOutlierSet = points(~inliers, :); %points where logical array is not true
        bestcandidates=candidates;
        
        %these two lines find a nice set of endpoints for plotting the best
        %fit line
        projectedCoordinate = diffs(inliers, :)*v/norm(v);
        bestEndPoints = [min(projectedCoordinate); max(projectedCoordinate)]*v'/norm(v) + repmat(candidates(2, :), [2, 1]);
    end

end

if isempty(bestEndPoints)
    m= NaN;
    b= NaN;
    bestEndPoints=[NaN,NaN;NaN,NaN];
    fitline_coefs=[m b];
    return;
end
    
%Find the coefficients for the best line
m=diff(bestEndPoints(:,2))/diff(bestEndPoints(:,1));
b=bestEndPoints(1,2)-m*bestEndPoints(1,1);
fitline_coefs=[m b];

if visualize==1

%plot the polar data as verification
figure(1)
polarplot(deg2rad(theta_clean),r_clean,'ks','MarkerSize',6,'MarkerFaceColor','m')
title('Visualization of Polar Data')

figure(2)
plot(x,y,'ks')
title('Scan Data- Clean')
xlabel('[m]')
ylabel('[m]')

%Now we need to plot our results
figure(3)
plot(bestInlierSet(:,1), bestInlierSet(:,2), 'ks')
hold on
plot(bestOutlierSet(:,1),bestOutlierSet(:,2),'bs')
plot(bestEndPoints(:,1), bestEndPoints(:,2), 'r')
legend('Inliers','Outliers','Best Fit','location','northwest')
title(['RANSAC with d=' num2str(d) ' and n=' num2str(n)])
xlabel('[m]')
ylabel('[m]')
% Create textbox
annotation(figure(3),'textbox',...
    [0.167071428571429 0.152380952380952 0.25 0.1],...
    'String',{'Number of Inliers:' num2str(size(bestInlierSet,1))},...
    'FitBoxToText','off');
end
end