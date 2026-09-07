function [theta1, theta2, theta3] = inverse_kinematics_RA(x, y, z, L1, L2, L3)
    L2_eff = L2 + L3 % accounting for end effector length
    theta1 = rad2deg(atan2(y, x)); % base joint angle
    
    % defining additional parameters for derivation
    r = sqrt(x^2 + y^2);
    rho = sqrt(r^2 + z^2);
    alpha = atan2(z, r);
    
    D = (r^2 + z^2 + L2_eff^2 - L1^2)/(2*L2_eff);
    
    % check reachability
    if abs(D/rho) > 1
        error('Target is outside reachable workspace');
    end

    phi1 = alpha + acos(D/rho);
    phi2 = alpha - acos(D/rho);
    
    phi = [phi1, phi2];

    theta2 = zeros(1,2);
    theta3 = zeros(1,2);
    
    for i = 1:2
        % multiple solutions covered by atan2 function
        theta2(i) = atan2( ...
            L2_eff*sin(phi(i)) - z, ...
            r - L2_eff*cos(phi(i)) );
        
        % multiple solutions accounted for by theta2
        theta3(i) = theta2(i) + phi(i);
    end
    
    % conversion to radians
    theta2 = rad2deg(theta2)
    theta3 = rad2deg(theta3)
end

% example test position (0.1, 0.1, 0.1)
inverse_kinematics_RA(0.1, 0.1, 0.1, 0.095, 0.1375, 0.04)