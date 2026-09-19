clear all; close all; clc;
%% 1. Parameter Fisik Banner & Robot
banner_size = 2.1;         
cell_size = 0.3;           
robot_diam = 0.3;          
R_robot = robot_diam / 2;  
ban_h = 0.082; % Tinggi ban 8.2cm
safety_margin = 0.10;      
total_safe_dist = R_robot + safety_margin;

% Parameter Sensor
num_sensors = 8;
max_sensor_range = 1.0;    
fov_angle = deg2rad(25);   
sensor_angles = linspace(0, 2*pi - (2*pi/num_sensors), num_sensors);

% Posisi Start Tetap
x_start = 0.0; y_start = 0.0;
x = x_start; y = y_start; theta_robot = 0;
position_accuracy = 0.05;

% Parameter APF
zeta = 5.0; eta = 1.0; dstar = 0.5; Qstar = total_safe_dist + 0.1;
v_max = 0.25; dT = 0.1; t_max = 1500;

%% 2. Inisialisasi Grafik untuk Input Interaktif
figure(1); set(gcf, 'Position', [50, 50, 900, 900], 'Color', 'w');
axis equal; grid on; hold on;
xlim([-0.3, banner_size + 0.3]); ylim([-0.3, banner_size + 0.3]);
rectangle('Position',[0 0 banner_size banner_size], 'EdgeColor', 'k', 'LineWidth', 2);
xticks(0:cell_size:banner_size); yticks(0:cell_size:banner_size);
title('1. KLIK UNTUK MENENTUKAN TITIK GOAL (TUJUAN)');
% --- A. INPUT GOAL ---
[x_goal, y_goal] = ginput(1);
plot(x_goal, y_goal, 'bp', 'MarkerSize', 15, 'MarkerFaceColor', 'b');
text(x_goal + 0.05, y_goal + 0.05, sprintf('Goal: %.2f, %.2f', x_goal, y_goal), 'Color', 'b', 'FontWeight', 'bold');
% --- B. INPUT OBSTACLE ---
title('2. KLIK UNTUK MENAMBAH OBSTACLE, TEKAN ENTER JIKA SELESAI');
make_box = @(cx, cy, side) [linspace(cx-side/2, cx+side/2, 20), linspace(cx+side/2, cx+side/2, 20), linspace(cx+side/2, cx-side/2, 20), linspace(cx-side/2, cx-side/2, 20);
                           linspace(cy-side/2, cy-side/2, 20), linspace(cy-side/2, cy+side/2, 20), linspace(cy+side/2, cy+side/2, 20), linspace(cy+side/2, cy-side/2, 20)];
obst = {};
count = 0;
while true
   [ox, oy, button] = ginput(1);
   if isempty(ox), break; end % Berhenti jika tekan Enter
   count = count + 1;
   obst{count} = make_box(ox, oy, 0.15);
   fill(obst{count}(1,:), obst{count}(2,:), [0.8 0.2 0.2], 'EdgeColor', 'none');
   text(ox + 0.1, oy, sprintf('Obs %d: %.2f, %.2f', count, ox, oy), 'FontSize', 8, 'Color', [0.5 0 0]);
end

%% 3. Definisi Bentuk Robot
rot_offset = pi/8;
angles = linspace(0, 2*pi, 9) + rot_offset;
robot_shape_x = R_robot * cos(angles);
robot_shape_y = R_robot * sin(angles);

%% 4. Loop Utama Navigasi
t = 1; X = x; Y = y;
title('SIMULASI BERJALAN...');
while norm([x_goal y_goal] - [x y]) > position_accuracy && t < t_max
   % --- LOGIKA APF ---
   dist_goal = norm([x y] - [x_goal y_goal]);
   nablaU_att = (dist_goal <= dstar) * zeta * ([x y] - [x_goal y_goal]) + ...
                (dist_goal > dstar) * (dstar/dist_goal * zeta * ([x y] - [x_goal y_goal]));
   nablaU_rep = [0 0];
   all_obst_points = cell2mat(obst);
   for i = 1:length(obst)
       [idx, dist_obs] = dsearchn(obst{i}', [x y]);
       if dist_obs <= Qstar
           potensi = (eta * (1/Qstar - 1/dist_obs) * 1/dist_obs^2);
           nablaU_rep = nablaU_rep + potensi * ([x y] - obst{i}(:,idx)');
       end
   end
   % --- UPDATE GERAK (HOLONOMIC) ---
   nablaU = nablaU_att + nablaU_rep;
   theta_move = atan2(-nablaU(2), -nablaU(1));
   v_ref = min(norm(nablaU), v_max);
   x = x + v_ref * cos(theta_move) * dT;
   y = y + v_ref * sin(theta_move) * dT;
   t = t + 1; X(t) = x; Y(t) = y;
   
   % --- VISUALISASI ---
   cla; hold on; grid on; axis equal;
   xlim([-0.3, banner_size + 0.3]); ylim([-0.3, banner_size + 0.3]);
   rectangle('Position',[0 0 banner_size banner_size], 'EdgeColor', 'k', 'LineWidth', 2);
   xticks(0:cell_size:banner_size); yticks(0:cell_size:banner_size);
  
   % Gambar Goal & Jalur
   plot(x_goal, y_goal, 'bp', 'MarkerSize', 15, 'MarkerFaceColor', 'b');
   text(x_goal + 0.05, y_goal + 0.05, sprintf('Goal: %.2f, %.2f', x_goal, y_goal), 'Color', 'b', 'FontSize', 8);
   plot(X, Y, '-bo', 'LineWidth', 2, 'MarkerSize', 4, 'MarkerFaceColor', 'b');
  
   % Gambar Obstacles & Labelnya
   for i = 1:length(obst)
       fill(obst{i}(1,:), obst{i}(2,:), [0.8 0.2 0.2], 'EdgeColor', 'none');
       % Titik tengah obstacle untuk label teks
       mid_x = mean(obst{i}(1,:)); mid_y = mean(obst{i}(2,:));
       text(mid_x + 0.1, mid_y, sprintf('Obs: %.2f, %.2f', mid_x, mid_y), 'FontSize', 7, 'Color', [0.5 0 0]);
   end
  
   % SENSOR GELOMBANG & TEKS JARAK
   for s = 1:num_sensors
       base_ang = theta_robot + sensor_angles(s);
       sweep_ang = linspace(base_ang - fov_angle/2, base_ang + fov_angle/2, 10);
       actual_range = max_sensor_range;
       for ang = sweep_ang
           temp_x = x + max_sensor_range * cos(ang); temp_y = y + max_sensor_range * sin(ang);
           check_pts_x = linspace(x, temp_x, 15); check_pts_y = linspace(y, temp_y, 15);
           for cp = 1:15
               if isempty(all_obst_points), break; end
               [~, d_check] = dsearchn(all_obst_points', [check_pts_x(cp) check_pts_y(cp)]);
               if d_check < 0.05
                   d_hit = norm([check_pts_x(cp)-x, check_pts_y(cp)-y]);
                   if d_hit < actual_range, actual_range = d_hit; end
                   break;
               end
           end
       end
       patch([x, x + actual_range * cos(sweep_ang), x], [y, y + actual_range * sin(sweep_ang), y], ...
             [0 0.8 0], 'FaceAlpha', 0.15, 'EdgeColor', 'none');
       text(x + (actual_range+0.1)*cos(base_ang), y + (actual_range+0.1)*sin(base_ang), ...
            [num2str(round(actual_range*100)), 'cm'], 'FontSize', 7, 'HorizontalAlignment', 'center');
   end
  
   % GAMBAR BAN (POSISI X)
   wheel_angles = [pi/4, 3*pi/4, 5*pi/4, 7*pi/4];
   for wa = wheel_angles
       wx = x + R_robot * cos(wa); wy = y + R_robot * sin(wa);
       wheel_w = 0.04; wheel_h = ban_h;
       R_wheel = [cos(wa) -sin(wa); sin(wa) cos(wa)];
       wheel_pts = R_wheel * [ -wheel_w/2 wheel_w/2 wheel_w/2 -wheel_w/2; -wheel_h/2 -wheel_h/2 wheel_h/2 wheel_h/2 ];
       fill(wheel_pts(1,:) + wx, wheel_pts(2,:) + wy, [0.1 0.1 0.1]);
   end
   % GAMBAR ROBOT (Kuning, Axis Tengah, Koordinat)
   fill(robot_shape_x + x, robot_shape_y + y, 'y', 'FaceAlpha', 0.5, 'EdgeColor', 'k');
   plot([x-R_robot, x+R_robot], [y, y], 'k:', 'LineWidth', 0.5);
   plot([x, x], [y-R_robot, y+R_robot], 'k:', 'LineWidth', 0.5);
   text(x, y - 0.25, sprintf('X:%.2f\nY:%.2f', x, y), 'FontSize', 8, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
  
   % Penanda FRONT
   patch([R_robot-0.03, R_robot+0.05, R_robot-0.03] + x, [-0.03, 0, 0.03] + y, 'r');
  
   drawnow;
end

