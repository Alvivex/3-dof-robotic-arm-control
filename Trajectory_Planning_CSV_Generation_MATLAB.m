
% link lengths
L1 = 0.095;
L2 = 0.1375;
L3 = 0.04;

% initial and final positions
x0 = 0; 
y0 = 0.1775; 
z0 = -0.095;

xf = 0.065;
yf = 0.165; 
zf = 0.06;

% trajectory generation frequency
tf = 5;      % total motion time (seconds)
dt = 0.1;    % timestep

t = 0:dt:tf;
N = length(t);

% generate trajectory points
x = x0 + (3/(tf^2))*(xf-x0).*t.^2 - (2/(tf^3))*(xf-x0).*t.^3;
y = y0 + (3/(tf^2))*(yf-y0).*t.^2 - (2/(tf^3))*(yf-y0).*t.^3;
z = z0 + (3/(tf^2))*(zf-z0).*t.^2 - (2/(tf^3))*(zf-z0).*t.^3;

% storage for joint angles
theta1_all = zeros(N,1);
theta2_all = zeros(N,1);
theta3_all = zeros(N,1);

% inverse kinematics for each trajectory point
for i = 1:N
    
    [theta1, theta2, theta3] = inverse_kinematics_RA(x(i),y(i),z(i),L1,L2,L3);
    
    theta1_all(i) = theta1;
    theta2_all(i) = theta2(1); % choose solution branch 1
    theta3_all(i) = theta3(1);

end

% combine time and joint angles
joint_trajectory = [t' theta1_all theta2_all theta3_all];

% save csv file
writematrix(joint_trajectory,'C:\Users\turbo\OneDrive\Documents\University Stuff\2nd Year\Term 2\Introduction to Robotics\Group Coursework\Trajectory Planning\joint_trajectory.csv');

disp('CSV file saved: joint_trajectory.csv')


% inverse kinematics from previous section
function [theta1, theta2, theta3] = inverse_kinematics_RA(x, y, z, L1, L2, L3)

    L2_eff = L2 + L3;

    theta1 = rad2deg(atan2(y, x));

    % Additional parameters
    r = sqrt(x^2 + y^2);
    rho = sqrt(r^2 + z^2);
    alpha = atan2(z, r);

    D = (r^2 + z^2 + L2_eff^2 - L1^2) / (2*L2_eff);

    % Reachability check
    if abs(D/rho) > 1
        error('Target is outside reachable workspace');
    end

    phi1 = alpha + acos(D/rho);
    phi2 = alpha - acos(D/rho);

    phi = [phi1, phi2];

    theta2 = zeros(1,2);
    theta3 = zeros(1,2);

    for i = 1:2
        
        theta2(i) = atan2( ...
            L2_eff*sin(phi(i)) - z, ...
            r - L2_eff*cos(phi(i)) );

        theta3(i) = theta2(i) + phi(i);
    end

    theta2 = rad2deg(theta2);
    theta3 = rad2deg(theta3);

end