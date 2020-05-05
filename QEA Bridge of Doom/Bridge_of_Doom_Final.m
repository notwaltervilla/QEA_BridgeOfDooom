function starterCodeForBridgeOfDoomQEA2020(dataset)
% define u explicitly to avoid error when using sub functions
% see: https://www.mathworks.com/matlabcentral/answers/268580-error-attempt-to-add-variable-to-a-static-workspace-when-it-is-not-in-workspace
u = [];
% u will be our parameter
syms u;
% additional parameters are defined
% u is a function of t, using b as a scalar
t = sym('t');
b = sym('b');

% this is the equation of the bridge
R = 4*[0.396*cos(2.65*(u+1.4));...
       -0.99*sin(u+1.4);...
       0];
   
% the bridge equation is separated into its different components
ri=4*(0.396*cos(2.65*(u+1.4)));
rj=4*(-0.99*sin(u+1.4));
rk=0*u;

% components are restructured into a matrix
r = [ri,rj,rk];

% t (time) and b (frequency modulation/ scalar for time) are substituted in
% for u
r = subs(r,[u],[b*t]);

% calculate T-hat (unit tangent vector)
T_hat=diff(r,t)/norm(diff(r,t));

% calculate velocity
velocity=diff(r,t);
velocity=simplify(velocity);

% define speed as magnitude of velocity
speed=norm(velocity);
speed=simplify(speed);

% calculate angular velocity
d_theta=diff(T_hat,t);
omega=simplify(cross(T_hat,d_theta));
omega=omega(:,3);

% tangent vector (for resetting Neato position)
T = diff(R);

% normalized tangent vector (for resetting Neato position)
That = T/norm(T);

pub = rospublisher('raw_vel');

% stop the robot if it's going right now
stopMsg = rosmessage(pub);
stopMsg.Data = [0 0];
send(pub, stopMsg);

% reset Neato position
bridgeStart = double(subs(R,u,0));
startingThat = double(subs(That,u,0));
placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), startingThat(2));

% wait a bit for robot to fall onto the bridge
pause(2);

% time to drive!!
message = rosmessage(pub);
r1 = rosrate(20.7);
reset(r1);

% wheel distance and b are defined
d = 0.235;
b1 = 0.22;

% encoder dataset is imported
% format = [timestamp, positionLeft, positionRight, AccelX, AccelY, AccelZ];
%{
timestamp = dataset(:,1);
position_left_encoder = dataset(:,2);
position_right_encoder = dataset(:,3);
left_pos_func = r - (d/2);
position_left_real = subs(left_pos_func, [b], [b1]);
position_right_real = r + (d/2);
hold off
hold on
for i = 1:141
    elapsed = timestamp(i);
    plot(position_left_encoder(i), elapsed, "b.")
    plot(subs(position_left_real, [t], [elapsed]), elapsed, "r.")
end
%}


% plotting speed and angular velocity vs time, 
% and plotting calculated and encoder reconstructed positions
%{
hold off
hold on
encoder_time = dataset(:,1);
pos_L = dataset(:,2);
pos_R = dataset(:,3);
dt = diff(encoder_time);
d_pos_L = diff(pos_L);
d_pos_R = diff(pos_R);
encoder_VL = d_pos_L./dt;
encoder_VR = d_pos_R./dt;
Lin_speed = (encoder_VL + encoder_VR)./2;
encoder_omega = (encoder_VR - encoder_VL)./d;

encoder_pos = zeros(length(encoder_time),2);
encoder_thet = zeros(length(encoder_time),1);
for i = 1:length(dt)
    encoder_pos(i+1,1) = encoder_pos(i,1) + Lin_speed(i)*cos(encoder_thet(i))*dt(i);
    encoder_pos(i+1,2) = encoder_pos(i,2) + Lin_speed(i)*sin(encoder_thet(i))*dt(i);
    encoder_thet(i+1) = encoder_thet(i) + encoder_omega(i)*dt(i);
end

%
Vl=(simplify(speed-(omega*d/2)));
Vr=(simplify(speed+(omega*d/2)));
Vl=subs(Vl,b,b1);
Vr=subs(Vr,b,b1);
Vl_mat = [];
Vr_mat = [];
for i = 1:141
    Vl_mat(i) = transpose(double(subs(Vl,t,encoder_time(i))));
    Vr_mat(i) = transpose(double(subs(Vr,t,encoder_time(i))));
end
V_calc = (Vl_mat + Vr_mat)./2;
calc_pos = zeros(141,2);
theta_calc = zeros(141,1);
calc_omega = (Vr_mat - Vl_mat)./d;
for i = 1:140
    calc_pos(i+1,1) = calc_pos(i,1) + V_calc(i)*cos(theta_calc(i))*dt(i);
    calc_pos(i+1,2) = calc_pos(i,2) + V_calc(i)*sin(theta_calc(i))*dt(i);
    theta_calc(i+1) = theta_calc(i) + calc_omega(i)*dt(i);
end
%}
plot(encoder_pos(:,1), encoder_pos(:,2), "--")
plot(calc_pos(:,1), calc_pos(:,2), "-")
title('Calculated vs Measured Reconstructed Positions')
xlabel('X-position (meters)')
ylabel('Y-position (meters)')
for i = 1:20:141
    quiver(calc_pos(i,1),calc_pos(i,2),cos(theta_calc(i)),sin(theta_calc(i)),"k")
end
for i = 1:15:141
    quiver(encoder_pos(i,1),encoder_pos(i,2),cos(encoder_thet(i)),sin(encoder_thet(i)), "r")
end
legend({'Encoder Data','Calculated Data'},'Location','southwest')

N_speed = subs(speed,[b],[b1]);
N_omega = subs(omega,[b],[b1]);
N_start = rostime('now');

N_speed_mat = [];
N_omega_mat = [];

for i = 1:141
    N_current = rostime('now');
    N_elapsed = N_current - N_start;
    N_speed1 = double(subs(N_speed,[t],[encoder_time(i)]));
    N_omega1 = double(subs(N_omega,[t],[encoder_time(i)]));
    N_speed_mat(i) = N_speed1;
    N_omega_mat(i) = N_omega1;
end
%plot(Lin_speed, "--")
%plot(transpose(N_speed_mat), "-")
%title('Linear Speed vs Time')
%legend({'Encoder Data','Calculated Data'},'Location','southwest')
%xlabel('Time (seconds)')
%ylabel('Linear Speed (m/s)')

%plot(encoder_omega, "--")
%plot(transpose(N_omega_mat), "-")
%title('Angular Velocity vs Time')
%legend({'Encoder Data','Calculated Data'},'Location','southwest')
%xlabel('Time (seconds)')
%ylabel('Angular Velocity (rad/sec)')
%}

% plotting VL and VR vs time and running simulation
%{
% VL and VR are defined
Vl=(simplify(speed-(omega*d/2)));
Vr=(simplify(speed+(omega*d/2)));

% finding left and right wheel velocities from encoder data
encoder_time = dataset(:,1);
pos_L = dataset(:,2);
pos_R = dataset(:,3);
dt = diff(encoder_time);
d_pos_L = diff(pos_L);
d_pos_R = diff(pos_R);
encoder_VL = d_pos_L./dt;
encoder_VR = d_pos_R./dt;

% plotting encoder wheel velocities
hold off
hold on
plot(encoder_VL, "b--")
plot(encoder_VR, "r--")

% b-value is substituted in for b
Vl=subs(Vl,b,b1);
Vr=subs(Vr,b,b1);

calc_left = [];
calc_right = [];
% plotting calculated wheel velocities
for i = 1:141
    left = double(subs(Vl,t,encoder_time(i)));
    right = double(subs(Vr,t,encoder_time(i)));
    calc_left(i) = left;
    calc_right(i) = right;
end
transpose(calc_left);
plot(transpose(calc_left), "b-")
plot(transpose(calc_right), "r-")
title('Left and Right wheel velocities vs Time')
legend({'Left Wheel Encoder','Right Wheel Encoder','Left Wheel Calculated','Right Wheel Calculated'},'Location','southwest')
xlabel('Time (seconds)')
ylabel('Wheel velocities vs time (m/s)')
%}

%{
% b-value is substituted in for b
Vl=subs(Vl,[b],[b1]);
Vr=subs(Vr,[b],[b1]);
start = rostime('now')
for i = 1:700
    % calculate elapsed time
    current = rostime('now');
    elapsed = current - start;
    
    % sending position data to Neato
    message.Data = [double(subs(Vl,[t],[elapsed.seconds])), double(subs(Vr,[t],[elapsed.seconds]))];
    send(pub, message);
    pause(0.09995);
end

% stop the Neato when movement is done
message.Data = [0 0];
send(pub, message);
%}

% For simulated Neatos only:
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
end