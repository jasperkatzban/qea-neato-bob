function generate_ascent(points_x,points_y,delta, s0)
% generates a contour surface, calculates an ideal gradient descent path,
% and plots relevant data.

% set equation vars, flags, and empty iterable objects
syms x y real;
bob_height = 4;
obstacle_height = 1;
target_weight = 1;
f = 0;
f_num = 0;

%define and plot points around BoB perimeter
b_rad = .3;
b_pos = [.75; -2.5];
C = linspace(0,2*pi,24);
b_x = b_rad.*cos(C)' + b_pos(1);
b_y = b_rad.*sin(C)' + b_pos(2);
plot(b_x,b_y);

% define grid of points to use with our vectors
[xx,yy] = meshgrid(-2:.05:3, -4:.05:2.5); 

% iterate over line segments and generate a single logarithmic contour 
% function from RANSAC fitted lines
for line=1:size(points_x,1)
    for n = 1:size(points_x,2)
        % set points as either walls (sources) or BoB (sink)
        if inpolygon(points_x(line,n),points_y(line,n),b_x,b_y)
            target_weight = -bob_height;
        else
            target_weight = obstacle_height;
        end
        f = f + target_weight*log((x-points_x(line,n))^2 + (y-points_y(line,n))^2);
        f_num = f_num + target_weight*log((xx-points_x(line,n)).^2 + (yy-points_y(line,n)).^2);
    end
end
% compute gradient field and plot contour and vector fields
[px,py] = gradient(f_num,.05,.05);
contour(xx,yy,f_num); 
quiver(xx,yy,px,py);

% set up for gradient descent path finder loop
gradfunc=gradient(f);
xPoint= 0;
yPoint= 0;
gradvect = double(subs(gradfunc, [x y], [xPoint yPoint]));
gradnorm = norm(gradvect);
gradunitvect = gradvect/gradnorm;
s = s0;
xPoints=[xPoint];
yPoints=[xPoint];
plot(xPoint,yPoint,'x');

% iterate over this loop until the gradient is steep enough to ensure we've
% made it to the BoB
while gradnorm<1000  
 xPointnew = xPoint + gradunitvect(1)*s;
 yPointnew = yPoint + gradunitvect(2)*s;
 gradvectnew=double(subs(gradfunc, [x y], [xPointnew yPointnew]));
 gradnorm = norm(gradvectnew);
 gradunitvect = gradvectnew/gradnorm;
 s = delta*s;
 xPoints = horzcat(xPoints,xPointnew);
 yPoints = horzcat(yPoints,yPointnew);
 xPoint=xPointnew;
 yPoint=yPointnew;
 gradvect=gradvectnew;
 plot([xPoints xPointnew],[yPoints yPointnew], 'b');
end

% plot the path
plot(xPoints,yPoints,'x','Color','b');
axis([-1.5 2.5 -3 1]);
axis equal;

% set up some vars for our output equation
f_order = 5;
f_fit_line = 0;
f_py = 0;
syms f_x u real;
f_px = u;

% polynomial fit a function to the generated gradient descent path
f_coefs = polyfit(xPoints,yPoints,f_order);

% pack coefficients of polynomial into a symbolic function
for nn=1:length(f_coefs)
    f_fit_line = [f_fit_line + f_coefs(nn)*f_x^(f_order-nn+1)];
    f_py = [f_py + f_coefs(nn)*u^(f_order -nn + 1)];
end

% attempt to pack function into object
% f_fit = [f_px;f_py;0*u];  

end