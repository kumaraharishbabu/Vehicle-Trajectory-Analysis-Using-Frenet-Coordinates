clc; clear;

%% Step 1: Load and arc-length parameterize centerline
centerline = readtable('polyline_coordinates_expanded.csv');
x_raw = centerline.x_coordinate;
y_raw = centerline.y_coordinate;

% Arc-length parameterization
centerline_dist = [0; cumsum(hypot(diff(x_raw), diff(y_raw)))];
ppx = spline(centerline_dist, x_raw');
ppy = spline(centerline_dist, y_raw');

% High-resolution centerline
t_fine = linspace(0, centerline_dist(end), 10000);
x_smooth = ppval(ppx, t_fine);
y_smooth = ppval(ppy, t_fine);

% Compute arc length values
dx = diff(x_smooth);
dy = diff(y_smooth);
ds = hypot(dx, dy);
s_vals = [0, cumsum(ds)];

%% Step 2: User input for sheet number
sheet_num = input('Enter the sheet number to process: ');
sheet_name = ['Sheet' num2str(sheet_num)];
fprintf('🔄 Processing: %s\n', sheet_name);

try
    trajectory = readtable('demo.xlsx', 'Sheet', sheet_name);
catch
    error('❌ Sheet %s not found in demo.xlsx.', sheet_name);
end

% Compute vehicle midpoints
x_center = (trajectory.X1 + trajectory.X2) / 2;
y_center = (trajectory.Y1 + trajectory.Y2) / 2;
vehicle_points = [x_center, y_center];

%% Step 3: Project using fminbnd
n = size(vehicle_points, 1);
s_frenet = zeros(n, 1);
d_frenet = zeros(n, 1);
x_proj_all = zeros(n, 1);
y_proj_all = zeros(n, 1);
normal_all = zeros(n, 2);

for i = 1:n
    pt = vehicle_points(i, :);

    % Minimize squared distance
    dist_fun = @(t) (ppval(ppx, t) - pt(1)).^2 + (ppval(ppy, t) - pt(2)).^2;
    t_proj = fminbnd(dist_fun, t_fine(1), t_fine(end));

    x_proj = ppval(ppx, t_proj);
    y_proj = ppval(ppy, t_proj);
    x_proj_all(i) = x_proj;
    y_proj_all(i) = y_proj;

    dx_dt = ppval(fnder(ppx), t_proj);
    dy_dt = ppval(fnder(ppy), t_proj);
    tangent = [dx_dt, dy_dt] / norm([dx_dt, dy_dt]);
    normal = [-tangent(2), tangent(1)];
    normal_all(i,:) = normal;

    s_idx = find(t_fine >= t_proj, 1);
    if isempty(s_idx), s_idx = length(s_vals); end
    s_frenet(i) = s_vals(s_idx);

    displacement = pt - [x_proj, y_proj];
    d_frenet(i) = dot(displacement, normal);
end

%% Step 4: Save to per-sheet file
output_filename = sprintf('frenet_sheet_%d.csv', sheet_num);
frenet_table = table(s_frenet, d_frenet, ...
    'VariableNames', {'s_arc_length', 'd_lateral_offset'});
writetable(frenet_table, output_filename);
fprintf('✅ Frenet coordinates saved to %s\n', output_filename);

%% Step 5: Reconstruction error check
x_recon = x_proj_all + d_frenet .* normal_all(:,1);
y_recon = y_proj_all + d_frenet .* normal_all(:,2);
recon_error = sqrt((x_recon - vehicle_points(:,1)).^2 + ...
                   (y_recon - vehicle_points(:,2)).^2);

fprintf('📏 Mean Reconstruction Error: %.2f pixels\n', mean(recon_error));
fprintf('📏 Max Reconstruction Error: %.2f pixels\n', max(recon_error));

%% Step 6: PLOT 1 - Vehicle Trajectory + Centerline (No projections yet)
figure('Name','Initial Vehicle Trajectory and Centerline');
hold on; axis equal; set(gca, 'YDir', 'reverse');
plot(x_smooth, y_smooth, 'r-', 'LineWidth', 2, 'DisplayName', 'Centerline');
plot(vehicle_points(:,1), vehicle_points(:,2), 'b.-', 'DisplayName', 'Vehicle Trajectory');
legend('Location', 'southwest');
title(sprintf('Sheet %d: Initial Vehicle Trajectory vs Centerline', sheet_num));
xlabel('X (pixels)');
ylabel('Y (pixels)');

%% Step 7: PLOT 2 - Vehicle Trajectory, Centerline & Projections
figure('Name','Vehicle Trajectory and Projections', 'Position', [100, 100, 1600, 900]);
hold on; axis equal; set(gca, 'YDir', 'reverse');
plot(x_smooth, y_smooth, 'r-', 'LineWidth', 2, 'DisplayName', 'Centerline');
plot(vehicle_points(:,1), vehicle_points(:,2), 'b.-', 'DisplayName', 'Vehicle Trajectory');

for i = 1:n
    plot([vehicle_points(i,1), x_proj_all(i)], ...
         [vehicle_points(i,2), y_proj_all(i)], 'g--', 'HandleVisibility','off');
end

plot(x_proj_all, y_proj_all, 'ko', 'MarkerSize', 4, 'DisplayName', 'Projection Points');

% Add padding to view
margin = 200;
x_min = min([vehicle_points(:,1); x_proj_all]) - margin;
x_max = max([vehicle_points(:,1); x_proj_all]) + margin;
y_min = min([vehicle_points(:,2); y_proj_all]) - margin;
y_max = max([vehicle_points(:,2); y_proj_all]) + margin;
axis([x_min x_max y_min y_max]);

legend('Location', 'southwest');
title(sprintf('Sheet %d: Vehicle Trajectory & Projections to Centerline', sheet_num));
xlabel('X (pixels)');
ylabel('Y (pixels)');


%% Step 8: PLOT 3 - d vs s in Frenet Frame
figure('Name','Frenet Frame Trajectory');
plot(s_frenet, d_frenet, 'b-', 'LineWidth', 1.5);
xlabel('s (arc length in pixels)');
ylabel('d (lateral offset in pixels)');
title(sprintf('Sheet %d: Lateral Offset (d) vs Arc Length (s)', sheet_num));
grid on;

