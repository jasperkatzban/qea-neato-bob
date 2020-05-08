function plotData(filepath)
% Function to compute theoretical outcomes and compare with experimental
% results by plotting both.

% load in the experimental data set if it exists
fprintf('Waiting for data file')
while isfile(filepath) == false
    if isfile(filepath)
        break;
    end
    fprintf('.')
    pause(1)
end

disp('...Found Data');

file = load(filepath, 'dataset');
dataset = file.dataset;

% define u as our main parameter and R as our curve equation
syms u R

beta = 1/3;

ri = 4*(0.3960*cos(2.65*(beta * u + 1.4)));
rj = 4*(-0.99*sin(beta * u + 1.4));
r=[ri,rj,0];

% take the derivative of the position with respect to u
dr=diff(r,u);

%To find the unit tangent we divide by the magnitude of r'=dr. To make this
%result look nice, we want to add some assumptions
assume(R,{'real','positive'});
assume(u,{'real','positive'});

%despite these assumptions, you will see that the unit tangent vector is a
%bit difficult to read (note I use the norm function, not abs)
T_hat_ugly=dr./norm(dr);

%We can use the simplify function to make things look nice
T_hat=simplify(T_hat_ugly);

%Next, we want to find the unit normal vector
dT_hat=diff(T_hat,u);
N_hat=dT_hat/norm(dT_hat);
N_hat=simplify(N_hat);

% break up the data into useful variables, we only want the first 3 columns
% of time and left/right wheel encoders
t = dataset(:,1);
LT_L = dataset(:,2);
LT_R = dataset(:,3);

% automatically align the data with the start frame
% effectively waiting until the robot spins up to get accurate readings
start_index = 1;
for k=1:length(LT_L)
    if mean([LT_L(k),LT_R(k)]) - mean([LT_L(1),LT_R(1)]) > 1.35
        start_index = k;
        break;
    end
end

t = t(1:end-(start_index-1));
LT_L = LT_L(start_index:end);
LT_R = LT_R(start_index:end);

%If we wish to visualize this curve, we can make substitutions into our
%functions. 

R_num = 1;
u_num = t;

v = diff(r,u);
dT_hat = diff(T_hat, u);
omega = cross(T_hat, dT_hat);

%we will also convert to a number of type double
for n=1:length(u_num)
    r_num(n,:)=double(subs(r,[R, u],[R_num, u_num(n)]));
    T_hat_num(n,:)=double(subs(T_hat,[R, u],[R_num, u_num(n)]));
    N_hat_num(n,:)=double(subs(N_hat,[R, u],[R_num, u_num(n)]));
    
    speed_num(n) = norm(double(subs(v,[u],[u_num(n)])));
    omega_num_temp = double(subs(omega,[u],[u_num(n)]));
    omega_num(n) = omega_num_temp(3);
    
    VL_num(n) = speed_num(n) - omega_num(n)*(0.235/2);
    VR_num(n) = speed_num(n) + omega_num(n)*(0.235/2);
end

% calculate velocities
dt=diff(t);
dL=diff(LT_L);
dR=diff(LT_R);

vL=dL./dt;
vR=dR./dt;

% do some filtering to smooth out the data
windowSize = 10; 
b = (1/windowSize)*ones(1,windowSize); a = 1;
vL_filt=filter(b,a,vL);
vR_filt=filter(b,a,vR);

% plot the theoretical data
% figure()
% movegui('center');
% plot(u_num,VL_num)
% hold on
% plot(u_num,VR_num)

% plot the experimental data

% plot(t(1:end-1),vL_filt,'--')
% plot(t(1:end-1),vR_filt,'--')
% 
% xlabel('Time [s]')
% ylabel('Velocity [m/s]')
% title('36.2 - Wheel Velocities')
% legend('Theo. L Wheel', 'Theo. R Wheel','Exp. L Wheel', 'Exp. R Wheel')
% hold off

% compute linear and angular velocities with given wheelbase
d=0.235;
V=(vL_filt+vR_filt)./2;
omega=(vR_filt-vL_filt)./d;

% % plot theoretical and experimental speed
% figure()
% movegui('east');
% hold on
% yyaxis left
% plot(u_num,speed_num)
% plot(t(1:end-1),V,'--')
% ylim([0,4])
% xlabel('Time [s]')
% ylabel('Linear Speed [m/s]')

% plot theoreticla and experimental angular velocity
% yyaxis right
% plot(u_num,omega_num)
% plot(t(1:end-1),omega,'--');
% ylabel('Angular Velocity [rad/s]')
% legend('Theo. Linear Speed', 'Exp. Linear Speed', 'Theo. Angular Velocity', 'Exp. Angular Velocity')
% title('36.3 - Speed vs Angular Velocity')
% hold off

% plot theoretical and experimental positions

% initial conditions
r=zeros(length(t),2);
theta=zeros(length(t),1); % vector is one shorter due to diff()

% map the encoder values to positional ones
for n=1:length(dt) 
    r(n+1,1)=r(n,1)+V(n)*cos(theta(n))*dt(n);
    r(n+1,2)=r(n,2)+V(n)*sin(theta(n))*dt(n);
    theta(n+1)=theta(n)+omega(n)*dt(n);
end

% plot the paths
r_num(:,2)=r_num(:,2)+4;
figure()
movegui('west');
plot(r_num(:,1),r_num(:,2))
hold on
plot(r(:,1),r(:,2),'--')

% plot the theoretical normals 
% for n=1:length(r_num)
%     % plot the curve and some unit tangent normals at specific points
%     if mod(n,10) == 0
%         quiver(r_num(n,1),r_num(n,2),T_hat_num(n,1),T_hat_num(n,2),'r') % plot the unit tangent
%         quiver(r_num(n,1),r_num(n,2),N_hat_num(n,1),N_hat_num(n,2),'b') % plot the unit normal
%     end
% end

% plot the experimental unit normals
% for k=1:10:length(r)
%     if mod(k,1) == 0
%         quiver(r(k,1),r(k,2),cos(theta(k)),sin(theta(k)))
%     end
% end

axis([-1.5 2.5 -3 1]);
axis equal
xlabel('X Position [m]')
ylabel('Y Position [m]')
legend('Theoretical Robot Position','Experimental Robot Position')
title('36.1 - Bridge of Doom - Parametric Path')
hold off
end