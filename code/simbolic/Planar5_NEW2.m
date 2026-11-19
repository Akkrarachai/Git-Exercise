clear; clc; close all;

%% Step 1: Symbolic variables
syms theta2 theta5 Px Py real

%% Step 2: Link lengths
L1 = 4;
L2 = 2;
L3 = 2.5;
L4 = 2.5;
L5 = 2;

%% Step 3: Position of A and B
Ax = L2*cos(theta2);
Ay = L2*sin(theta2);

Bx = L1 + L5*cos(theta5);
By = L5*sin(theta5);

%% Step 4: Inverse kinematics equations
% Distance AP must equal L3
eq1 = (Px-Ax)^2+(Py-Ay)^2 == L3^2;
% Distance BP must equal L4
eq2 = (Px-Bx)^2+(Py-By)^2 == L4^2;

disp('Inverse Kinematics Equations;');
disp(eq1);
disp(eq2);

%% Step 5: Circular trajectory
syms t real
xc = 2.0;
yc = 2.5;
r  = 0.5;

Px_path = xc+r*cos(t);
Py_path = yc+r*sin(t);

% Plot desired circular path
figure;
fplot(Px_path,Py_path,[0 2*pi],'r--','LineWidth',2);
axis equal;
grid on;
xlabel('X Position');
ylabel('Y Position');
title('Desired Circular Trajectory of P');

%% Step 6: Analytical Inverse Kinematics
t_values = linspace(0, 2*pi, 240);

theta2_values = nan(size(t_values));
theta5_values = nan(size(t_values));

for k = 1:length(t_values)

    tk = t_values(k);

    pxk = xc + r*cos(tk);
    pyk = yc + r*sin(tk);

    % Left arm: O2 -> A -> P
    d2 = hypot(pxk, pyk);
    c2 = (d2^2 + L2^2 - L3^2)/(2*L2*d2);

    % Right arm: O5 -> B -> P
    dx5 = pxk - L1;
    d5 = hypot(dx5, pyk);
    c5 = (d5^2 + L5^2 - L4^2)/(2*L5*d5);

    % Check reachability
    if abs(c2) > 1 || abs(c5) > 1
        error('Point P is outside workspace at step %d',k);
    end

    % Select IK branches
    theta2_values(k) = atan2(pyk, pxk) + acos(c2);
    theta5_values(k) = atan2(pyk, dx5) - acos(c5);
end

%% Step 6.5: ปรับค่ามุมเริ่มต้น (เปลี่ยนกรอบอ้างอิงของมุม)
% กำหนดมุมของแกนอ้างอิง 0 องศาใหม่ เทียบกับแกน +X เดิม (หน่วย: องศา)
theta2_ref_deg = 90;   % ปรับมุมอ้างอิงเริ่มต้นของมอเตอร์ซ้าย O2 (องศา)
theta5_ref_deg = 90;   % ปรับมุมอ้างอิงเริ่มต้นของมอเตอร์ขวา O5 (องศา)

theta2_ref = deg2rad(theta2_ref_deg);
theta5_ref = deg2rad(theta5_ref_deg);

% เลื่อนกรอบอ้างอิงของมุม (เก็บในตัวแปรชื่อเดิมเพื่อส่งออกสู่ Simscape)
theta2_values = theta2_values - theta2_ref;
theta5_values = theta5_values - theta5_ref;

%% Step 8: Export Joint Angles for Simscape Driving Input

% 1. กำหนดเวลาจริงในการจำลอง (Simulation Duration) หน่วยเป็นวินาที (s)
T_total = 10; % กำหนดให้แขนกลเคลื่อนที่ครบ 1 รอบในเวลา 5 วินาที
time_vec = linspace(0, T_total, length(t_values))'; % เวกเตอร์เวลา [240x1]

% 2. จัดรูปแบบค่ามุมให้อยู่ในแนวตั้ง (Column Vector)
theta2_vec = theta2_values'; % มอเตอร์ซ้าย O2 (rad)
theta5_vec = theta5_values'; % มอเตอร์ขวา O5 (rad)

% 3. สร้าง Matrix รวมคู่สัญญาณกับเวลา [Time, Theta2, Theta5] (สำหรับบล็อก From Workspace)
simscape_input_matrix = [time_vec, theta2_vec, theta5_vec];

% 4. สร้างเป็น timeseries Object (รูปแบบมาตรฐานที่แนะนำสำหรับ Simulink)
theta2_ts = timeseries(theta2_vec, time_vec);
theta5_ts = timeseries(theta5_vec, time_vec);

% จับคู่ทั้งสองมุมเข้าด้วยกันใน timeseries เดียว
theta_pair_ts = timeseries([theta2_vec, theta5_vec], time_vec);

disp('--- ส่งออกข้อมูลมุมสำหรับ Simscape เข้าสู่ Workspace สำเร็จ ---');
disp('ตัวแปรที่ถูกสร้าง: simscape_input_matrix, theta2_ts, theta5_ts, theta_pair_ts');

%% Plot motor angles
figure;
plot(t_values,rad2deg(theta2_values),'b-', ...
    'LineWidth',2);
hold on;
plot(t_values,rad2deg(theta5_values),'r-', ...
    'LineWidth',2);

xlabel('Path parameter t (rad)');
ylabel('Motor angle (degrees)');
legend('theta2','theta5');
grid on;
title('Inverse Kinematics of Five-Bar Mechanism');

%% Step 7: Five-Bar Mechanism Animation

figure;
hold on;
grid on;
axis equal;

% Fixed joints
O2 = [0, 0];
O5 = [L1, 0];

% Desired circular trajectory
Px_path_values = xc + r*cos(t_values);
Py_path_values = yc + r*sin(t_values);

% Plot full desired circle
plot(Px_path_values, Py_path_values, ...
    'k--', 'LineWidth', 1.5);

% Initialize mechanism at first position (บวกมุมอ้างอิงกลับเพื่อให้พิกัดจริงไม่เปลี่ยน)
A = [L2*cos(theta2_values(1) + theta2_ref), ...
     L2*sin(theta2_values(1) + theta2_ref)];

B = [L1 + L5*cos(theta5_values(1) + theta5_ref), ...
     L5*sin(theta5_values(1) + theta5_ref)];

P = [Px_path_values(1), Py_path_values(1)];

% Ground link
plot([O2(1), O5(1)], [0, 0], ...
     'k-', 'LineWidth', 3);

% Create animated links
h2 = plot([O2(1), A(1)], ...
          [O2(2), A(2)], 'b-o', 'LineWidth', 3);

h3 = plot([A(1), P(1)], ...
          [A(2), P(2)], 'r-o', 'LineWidth', 3);

h4 = plot([B(1), P(1)], ...
          [B(2), P(2)], 'g-o', 'LineWidth', 3);

h5 = plot([O5(1), B(1)], ...
          [O5(2), B(2)], 'm-o', 'LineWidth', 3);

% End-effector point
hP = plot(P(1), P(2), 'ko', ...
          'MarkerFaceColor', 'k', ...
          'MarkerSize', 8);

% Actual trajectory traced by P
hTrace = plot(nan, nan, 'r-', 'LineWidth', 2);

xlabel('X Position');
ylabel('Y Position');
title('Planar Five-Bar Mechanism Animation');

xlim([-1, L1+1]);
ylim([-0.5, 4]);

%% Animation Loop (Repeat Automatically)

while ishandle(hP)

    for k = 1:length(t_values)

        % Stop if figure is closed
        if ~ishandle(hP)
            break;
        end

        % Current motor angles (แปลงกลับเป็นพิกัดโลกเพื่อคงรูปทรงและการเคลื่อนที่เดิม)
        t2 = theta2_values(k) + theta2_ref;
        t5 = theta5_values(k) + theta5_ref;

        % Joint A
        A = [L2*cos(t2), L2*sin(t2)];

        % Joint B
        B = [L1 + L5*cos(t5), ...
            L5*sin(t5)];
        fprintf('A = (%.2f, %.2f), B = (%.2f, %.2f)\n', ...
            A(1), A(2), B(1), B(2));
        % End effector P
        P = [Px_path_values(k), ...
            Py_path_values(k)];

        % Update Link 2 (Blue)
        set(h2, 'XData', [O2(1), A(1)], ...
            'YData', [O2(2), A(2)]);

        % Update Link 3 (Red)
        set(h3, 'XData', [A(1), P(1)], ...
            'YData', [A(2), P(2)]);

        % Update Link 4 (Green)
        set(h4, 'XData', [B(1), P(1)], ...
            'YData', [B(2), P(2)]);

        % Update Link 5 (Magenta)
        set(h5, 'XData', [O5(1), B(1)], ...
            'YData', [O5(2), B(2)]);

        % Update end effector P
        set(hP, 'XData', P(1), ...
            'YData', P(2));

        % Update traced trajectory
        set(hTrace, ...
            'XData', Px_path_values(1:k), ...
            'YData', Py_path_values(1:k));

        % Refresh animation
        drawnow;

        % Animation speed
        pause(0.015);

    end

end