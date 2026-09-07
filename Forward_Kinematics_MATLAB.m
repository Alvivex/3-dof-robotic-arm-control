% link lengths 
L1 = 0.095; 
L2 = 0.1775; 
 
% test angles 
test_angles = [ 
    45, 90, 90; 
    90, 45, 45; 
    90, 90, 0; 
    45, 45, 90; 
    30, 70, 60 
]; 
 
% dh transformation matrix 
DH = @(alpha, a, d, theta) [ 
    cos(theta),             -sin(theta),            0,              a; 
    sin(theta)*cos(alpha),  cos(theta)*cos(alpha),  -sin(alpha),    -d*sin(alpha); 
    sin(theta)*sin(alpha),  cos(theta)*sin(alpha),  cos(alpha),     d*cos(alpha); 
    0,                      0,                      0,              1 
]; 
 
% header display 
fprintf('%-6s | %-8s %-8s %-8s | %-10s %-10s %-10s | %-10s %-10s %-10s | %s\n', ... 
    'Test', 'θ1', 'θ2', 'θ3', ... 
    'X (DH)', 'Y (DH)', 'Z (DH)', ... 
    'X (form)', 'Y (form)', 'Z (form)', 'Match?'); 
fprintf('%s\n', repmat('-', 1, 110)); 
 
for k = 1:size(test_angles, 1) 
    t1 = deg2rad(test_angles(k, 1)); 
    t2 = deg2rad(test_angles(k, 2)); 
    t3 = deg2rad(test_angles(k, 3)); 
 
    % chain DH matrices 
    T01 = DH(0,     0,    0, t1); 
    T12 = DH(-pi/2, 0,    0, t2); 
    T23 = DH(pi,    L1,   0, t3); 
    T3e = DH(0,     L2,   0, 0); 
 
    T02 = T01 * T12; 
    T03 = T02 * T23; 
    T0e = T03 * T3e; 
 
    % joint positions 
    p0 = [0; 0; 0];          % base 
    p3 = T03(1:3, 4);        % joint 3 
 21
    pe = T0e(1:3, 4);        % end-effector 
    pos_dh = pe; 
 
    % simplified expressions (closed form) 
    c1 = cos(t1); s1 = sin(t1); 
    c2 = cos(t2); s2 = sin(t2); 
    c23 = cos(t2 - t3); 
    s23 = sin(t2 - t3); 
    px = L1*c1*c2 + L2*c1*c23; 
    py = L1*s1*c2 + L2*s1*c23; 
    pz = -L1*s2 - L2*s23; 
    pos_form = [px; py; pz]; 
 
    % match checking 
    err = norm(pos_dh - pos_form); 
    match = err < 1e-10; 
 
    fprintf('%-6d | %-8.0f %-8.0f %-8.0f | %-10.4f %-10.4f %-10.4f | %-10.4f %-10.4f %-10.4f | %s\n', ... 
        k, test_angles(k,:), ... 
        pos_dh(1), pos_dh(2), pos_dh(3), ... 
        pos_form(1), pos_form(2), pos_form(3), ... 
        string(match)); 
 
    % 3D Plotting 
    joints = [p0, p3, pe]; 
    X = joints(1, :); 
    Y = joints(2, :); 
    Z = joints(3, :); 
 
    figure('Name', sprintf('Config %d', k), 'Color', 'w'); 
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k'); 
    set(gcf, 'InvertHardcopy', 'on'); 
    plot3(X, Y, Z, '-', 'Color', [0.3 0.5 0.9], 'LineWidth', 3); 
    hold on; 
    plot3(X, Y, Z, 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r', ... 
        'MarkerEdgeColor', [0.2 0.2 0.8], 'LineWidth', 1.5); 
    hold off; 
 
    title(sprintf('\\theta_1 = %.2f, \\theta_2 = %.2f, \\theta_3 = %.2f\nP_{ee} = (%.4f, %.4f, %.4f)', ... 
        test_angles(k, 1), test_angles(k, 2), test_angles(k, 3), ... 
        pe(1), pe(2), pe(3)), ... 
        'FontSize', 14); 
    xlabel('X (m)', 'FontSize', 12); 
    ylabel('Y (m)', 'FontSize', 12); 
    zlabel('Z (m)', 'FontSize', 12); 
    grid on; 
    axis equal; 
    set(gca, 'ZDir', 'reverse'); 
    view([135 25]); 
end 