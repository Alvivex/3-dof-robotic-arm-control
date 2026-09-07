clear
clc

% load trajectory CSV
data = readmatrix('joint_trajectory.csv');

% columns: [time theta1 theta2 theta3]
theta1 = data(:,2);
theta2 = data(:,3);
theta3 = data(:,4);

N = length(theta1);

fprintf('\nCopy the following into your Arduino code:\n\n');

% print theta1 array
fprintf('float theta1_traj[] = {');
for i = 1:N
    fprintf('%.2f',theta1(i));
    if i ~= N
        fprintf(', ');
    end
end
fprintf('};\n\n');

% print theta2 array
fprintf('float theta2_traj[] = {');
for i = 1:N
    fprintf('%.2f',theta2(i));
    if i ~= N
        fprintf(', ');
    end
end
fprintf('};\n\n');

% print theta3 array
fprintf('float theta3_traj[] = {');
for i = 1:N
    fprintf('%.2f',theta3(i));
    if i ~= N
        fprintf(', ');
    end
end
fprintf('};\n\n');

% print number of points
fprintf('int numPoints = %d;\n\n',N);