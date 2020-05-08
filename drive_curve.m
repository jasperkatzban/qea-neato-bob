function drive_curve
% Drives the neato according to the given parametric curve equation

% initialize ros comms
rosshutdown(); rosinit('localhost',11311, 'NodeHost','host.docker.internal')

% define u as our parameter to vary with respect to time
u = [];
syms u

% specify dimensional scale parameter
s_fac = 9;

% specify time multiplier
m_num = 1/30;

% specify initial angle
init_angle = -77;

% setup data stream and stop all actions
pub = rospublisher('raw_vel');
stopMsg = rosmessage(pub);
stopMsg.Data = [0 0];
send(pub, stopMsg);

% place the robot at a predetermined location
[h_x,h_y] = pol2cart(deg2rad(init_angle),1)
placeNeato(0,0,h_x,h_y);

% wait a bit for robot to fall onto the bridge
pause(1);

% define time var and time multiplier symbolically
t = [];
m = [];
syms t m

% specify further for the interpreter
assume(m, {'real', 'positive'})
assume(t, {'real', 'positive'})

% define u as a function of time and our multipler
u = t * m;

% define position as separate components and pack into an equation
ri = s_fac * u;
rj = -((3624589426074917*u^5)/8796093022208 - (4955267413048043*u^4)/8796093022208 + (8458331297973835*u^3)/35184372088832 - (4843396272996467*u^2)/140737488355328 - (1794749873703973*u)/1125899906842624 - 2390711841125223/576460752303423488)
r=[ri,rj,u*0];

% attempt to unpack these values from a argument for the sake of
% flexability
% ri = func(1);
% rj = func(2);

% specify wheelbase of neato
d = 0.235;

% define wheel velocities as a function of curve and given wheelbase
[VL, VR] = generate_vels(r,d);

% set up wheel control publisher and message object
pub = rospublisher('/raw_vel');
msg = rosmessage(pub);

% set starting timestamp
start = rostime('now');

% loop communication until time is up.
while 1
    % compute elapsed time
    elapsed = rostime('now') - start;
    
    % set wheel speeds to calculated velocities
    speed_L = double(subs(VL, [m, t], [m_num, elapsed.seconds]));
    speed_R = double(subs(VR, [m, t], [m_num, elapsed.seconds]));
   
    % send the message
    msg.Data = [speed_L speed_R];
    send(pub, msg);
    
    % if the neato reaches the end, stop it
    if elapsed > m_num*210
        send(pub, stopMsg);
        break
    end
    
    % wait for data to transmit
    pause(0.05);
end

% Place the Neato in the specified x, y position and specified heading vector.
function placeNeato(posX, posY, headingX, headingY)
    svc = rossvcclient('gazebo/set_model_state');
    msg = rosmessage(svc);

    msg.ModelState.ModelName = 'neato_standalone';
    startYaw = atan2(headingY, headingX);
    quat = eul2quat([startYaw 0 0]);

    msg.ModelState.Pose.Position.X = posX;
    msg.ModelState.Pose.Position.Y = posY;
    msg.ModelState.Pose.Position.Z = 1.0;
    msg.ModelState.Pose.Orientation.W = quat(1);
    msg.ModelState.Pose.Orientation.X = quat(2);
    msg.ModelState.Pose.Orientation.Y = quat(3);
    msg.ModelState.Pose.Orientation.Z = quat(4);

    % put the robot in the appropriate place
    ret = call(svc, msg);
end

% generate instantaneous wheel velocities based on parametric equation
function [V_L, V_R] = generate_vels(r, d)
    v = diff(r, t);
    T = simplify(v ./ norm(v));
    dT = diff(T, t);
    w = simplify(cross(T, dT));
    V_T = simplify(dot(T, v));
    V_L = simplify(V_T - w(3) * d / 2);
    V_R = simplify(V_T + w(3) * d / 2);
end

end