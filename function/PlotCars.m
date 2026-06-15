% Plot directory
output_directory = 'C:/Users/monea/OneDrive/Documents/MATLAB/I-80-Emeryville-CA/vehicle-trajectory-data/0500pm-0515pm/kalman_filter/data/Picture/';

% Ensure the directory exists
if ~exist(output_directory, 'dir')
    mkdir(output_directory);
end

% Define plot properties
plot_width = 8.27;  % Width in inches
plot_height = 4.5;  % Height in inches
font_size = 10;   % Font size for labels and titles
line_width = 1.2; % Line width for plots

% Create figure with subplots
% figure('Units', 'inches', 'Position', [1, 1, 2 * plot_width, plot_height]);

figure('Units', 'inches', 'Position', [1, 1, 2 * plot_width / 3, plot_height * 1.3]);

% ( Row 1 )   1   2   3
% ( Row 2 )   4   5   6
% ( Row 3 )   7   8   9
% ( Row 4 )  10  11  12

%% **Left Subplot: Vehicle Position vs. Time (Original Plot)**
% subplot(1, 3, 1);
hold on;

% Extract L1 (OgLnID == 1) and R1 (OgLnID == 2)
L1_indices = find([L1.OgLnID] == 1);
R1_indices = find([L1.OgLnID] == 2);

% Colors for differentiation
color_R1 = 'r'; % Red for R1 (OgLnID == 2)
color_L1 = 'b'; % Blue for L1 (OgLnID == 1)
fill_color = [0.6 1 0.6]; % Light green color for the filled area


% Fill the region between 852 and 1302 (fix: use 4 corner points for a rectangle)
h_fill = fill([0 100 100 0], [1080 1080 1302 1302], fill_color, 'EdgeColor', 'none', 'FaceAlpha', 0.3);

% Plot all vehicles in R1 (OgLnID == 2) in red
h_R1 = gobjects(length(R1_indices), 1); % Store plot handles for legend
for j = 1:length(R1_indices)
    i = R1_indices(j);
    h_R1 = plot(L1(i).t, L1(i).str_X, 'Color', color_R1, 'LineWidth', line_width);
    % Add normal label near last position
    text(L1(i).t(end), L1(i).str_X(end), sprintf('V%d', L1(i).ID), ...
         'FontSize', font_size, 'Color', color_R1, 'VerticalAlignment', 'bottom');
end

% Plot all vehicles in L1 (OgLnID == 1) in blue
h_L1 = gobjects(length(L1_indices), 1); % Store plot handles for legend
for j = 1:length(L1_indices)
    i = L1_indices(j);
    h_L1 = plot(L1(i).t, L1(i).str_X, 'Color', color_L1, 'LineWidth', line_width);
    % Add normal label near last position
    text(L1(i).t(end), L1(i).str_X(end), sprintf('V%d', L1(i).ID), ...
         'FontSize', font_size, 'Color', color_L1, 'VerticalAlignment', 'bottom');
end

% Labels and title
xlabel('Time (s)', 'FontSize', font_size);
ylabel('Vehicle Position (m)', 'FontSize', font_size);
title('Vehicle Position vs. Time for R1 and L1', 'FontSize', font_size);

% Add grid and legend
grid on;
legend([h_fill h_R1(1) h_L1(1)], {'Merging Zone', 'R1 - OgLnID 2', 'L1 - OgLnID 1'}, ...
       'FontSize', font_size, 'Location', 'best');

annotation('textbox', [0.48, 0.0001, 0.05, 0.05], 'String', '(a)', 'FontSize', font_size + 5, ...
           'FontWeight', 'bold', 'EdgeColor', 'none');

% Increase font size of tick labels
set(gca, 'FontSize', 11);

% Save the plot
saveas(gcf, fullfile(output_directory, 'Vehicle_Position_vs_Real_Noise1v6.png'));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure('Units', 'inches', 'Position', [1, 1, 2 * plot_width / 3 * 2, plot_height * 1.5]);

%% **Right Subplot: Kalman Filter, Model, Normal, and Noisy Data**

% Define colors for different data
color_noisy = 'g--'; % Green dashed for noisy data
color_model = 'r-.'; % Red dash-dot for model data
color_KF = 'k-';     % Black solid for Kalman filter
color_real = 'b-';   % Blue solid for real (normal) data

% Select a sample vehicle (e.g., first vehicle from L1 for demonstration)
selected_LR_index = find([L1.OgLnID] == 2, 1); % You can change this index as needed
selected_LR_car = L1(selected_LR_index);

selected_L1_index = find([L1.ID] == 10000, 1);
selected_L1_car = L1(selected_L1_index);

% Select a target car from the str_v2v_data (e.g., v10000)
target_car_id = 'v10000'; % Replace with the ID of the car you want to compare

if isfield(selected_LR_car.str_v2v_data, target_car_id)
    target_car_v2v = selected_LR_car.str_v2v_data.(target_car_id);
    target_car_model = selected_LR_car.model_v2v.(target_car_id);
    target_car_KF = selected_LR_car.KF_v2v.(target_car_id);
else
    error('Target car ID %s not found in str_v2v_data.', target_car_id);
end

% Extract time and data
t = selected_L1_car.t;
x_noisy = target_car_v2v.X; % Noisy X
% y_noisy = target_car_v2v.Y; % Noisy Y
v_noisy = target_car_v2v.V; % Noisy V

x_model = target_car_model.z(1,:); % Model X
% y_model = target_car_model.z(2,:); % Model Y
% v_model = target_car_model.z(3,:); % Model V
v_model = target_car_model.z(2,:); % Model V

x_KF = target_car_KF.zhatstore(1,:); % Kalman Filter X
% y_KF = target_car_KF.zhatstore(2,:); % Kalman Filter Y
% v_KF = target_car_KF.zhatstore(3,:); % Kalman Filter V
v_KF = target_car_KF.zhatstore(2,:); % Kalman Filter V

x_real = selected_L1_car.str_X; % Real X
% y_real = selected_L1_car.str_Y; % Real Y
v_real = selected_L1_car.str_V; % Real V

credibility_t = selected_LR_car.str_cred.(target_car_id).t;  % Time for credibility score
credibility_score = selected_LR_car.str_cred.(target_car_id).credibility_score;
trustworthiness = selected_LR_car.str_cred.(target_car_id).trustworthy;

% **First Row: X vs. Time**
% subplot(3, 3, 2);
% hold on;

% subplot(3, 2, 1); 
subplot(2, 2, 1);
hold on;

plot(t, x_noisy, color_noisy, 'LineWidth', line_width);
plot(t, x_model, color_model, 'LineWidth', line_width);
plot(t, x_KF, color_KF, 'LineWidth', line_width);
plot(t, x_real, color_real, 'LineWidth', line_width);
ylabel('X (m)', 'FontSize', font_size);
title('X Position vs. Time of v10000', 'FontSize', font_size);
grid on;
legend({'Noisy', 'Model', 'KF', 'Real'}, 'FontSize', font_size, 'Location', 'best');

% **Second Row: Y vs. Time**
% subplot(3, 3, 5);
% hold on;

% subplot(3, 2, 3);
% hold on;
% 
% plot(t, y_noisy, color_noisy, 'LineWidth', line_width);
% plot(t, y_model, color_model, 'LineWidth', line_width);
% plot(t, y_KF, color_KF, 'LineWidth', line_width);
% plot(t, y_real, color_real, 'LineWidth', line_width);
% ylabel('Y (m)', 'FontSize', font_size);
% title('Y Position vs. Time of v10000', 'FontSize', font_size);
% grid on;

% **Third Row: V vs. Time**
% subplot(3, 3, 8);
% hold on;

% subplot(3, 2, 5);
subplot(2, 2, 3);
hold on;

plot(t, v_noisy, color_noisy, 'LineWidth', line_width);
plot(t, v_model, color_model, 'LineWidth', line_width);
plot(t, v_KF, color_KF, 'LineWidth', line_width);
plot(t, v_real, color_real, 'LineWidth', line_width);
xlabel('Time (s)', 'FontSize', font_size);
ylabel('Velocity (m/s)', 'FontSize', font_size);
title('Velocity vs. Time of v10000', 'FontSize', font_size);
grid on;

%% **Right Column: Credibility Score & Trustworthiness (2 Rows)**
% **Credibility Score vs. Time**
% subplot(2, 3, 3);
% hold on;

% subplot(3, 2, 2);
subplot(2, 2, 2);
hold on;

% Color
color_highlight = 'm'; % Bold magenta for the selected car

cred_fields = fieldnames(selected_LR_car.str_cred);
number_vehicles = length(cred_fields);

legend_entries = cell(number_vehicles, 1); % Initialize legend names

% Loop through all L1 cars (OgLnID == 2) and plot their credibility scores in light gray
for j = 1:length(fieldnames(selected_LR_car.str_cred))
    target_cars_id = cred_fields{j};

    credibility_ts = selected_LR_car.str_cred.(target_cars_id).t;  % Time for credibility score
    credibility_scores = selected_LR_car.str_cred.(target_cars_id).credibility_score;

    h(j) = plot(credibility_ts, credibility_scores, 'LineWidth', 0.5); % Light color for other cars
    legend_entries{j} = target_cars_id; % Store legend label
end

% Highlight the selected car (v10000) in bold
h_selected = plot(credibility_t, credibility_score, ...
    'Color', color_highlight, 'LineWidth', line_width);
legend_entries{end+1} = 'v10000 (Highlighted)'; % Add legend entry for v10000

xlabel('Time (s)', 'FontSize', font_size);
ylabel('Credibility Score', 'FontSize', font_size);
title('Credibility Score vs. Time', 'FontSize', font_size);
grid on;
leg = legend([h, h_selected], legend_entries, 'FontSize', font_size, 'Location', 'eastoutside');
% set(leg, 'Units', 'normalized', 'Position', [0.87, 0.55, 0.12, 0.3]); % Adjust position
set(leg, 'Units', 'normalized', 'Position', [0.83, 0.62, 0.12, 0.3]); % Adjust position

%% **Trustworthiness vs. Time**
% subplot(2, 3, 6);
% hold on;

% subplot(3, 2, 6);
subplot(2, 2, 4);
hold on;

% Plot trustworthiness of all L1 cars (OgLnID == 2)
for j = 1:length(fieldnames(selected_LR_car.str_cred))
    target_cars_id = cred_fields{j};

    credibility_ts = selected_LR_car.str_cred.(target_cars_id).t;  % Time for credibility score
    trustworthinesss = selected_LR_car.str_cred.(target_cars_id).trustworthy; 

    h_t(j) = plot(credibility_ts, trustworthinesss, 'LineWidth', 0.5); % Light color for other cars
    legend_entries{j} = target_cars_id; % Store legend label
end

% Highlight the selected car (v10000) in bold
h_selected_t = plot(credibility_t, trustworthiness, 'LineWidth', line_width, 'Color', color_highlight);
legend_entries{end+1} = 'v10000 (Highlighted)'; % Add legend entry for v10000

xlabel('Time (s)', 'FontSize', font_size);
ylabel('Trustworthiness', 'FontSize', font_size);
title('Trustworthiness vs. Time', 'FontSize', font_size);
grid on;
leg_t = legend([h_t, h_selected_t], legend_entries, 'FontSize', font_size);
% set(leg_t, 'Units', 'normalized', 'Position', [0.87, 0.15, 0.12, 0.3]); % Adjust position
set(leg_t, 'Units', 'normalized', 'Position', [0.83, 0.02, 0.12, 0.3]); % Adjust position

% Set label size
set(findall(gcf, '-property', 'FontSize'), 'FontSize', 14);

%% **Add (a), (b), (c) as Plot Annotations**
% annotation('textbox', [0.23, 0.0001, 0.05, 0.05], 'String', '(a)', 'FontSize', font_size + 2, ...
%            'FontWeight', 'bold', 'EdgeColor', 'none');
% 
% annotation('textbox', [0.51, 0.0001, 0.05, 0.05], 'String', '(b)', 'FontSize', font_size + 2, ...
%            'FontWeight', 'bold', 'EdgeColor', 'none');
% 
% annotation('textbox', [0.79, 0.0001, 0.05, 0.05], 'String', '(c)', 'FontSize', font_size + 2, ...
%            'FontWeight', 'bold', 'EdgeColor', 'none');

annotation('textbox', [0.28, 0.0001, 0.05, 0.05], 'String', '(b)', 'FontSize', font_size + 8, ...
           'FontWeight', 'bold', 'EdgeColor', 'none');

annotation('textbox', [0.73, 0.0001, 0.05, 0.05], 'String', '(c)', 'FontSize', font_size + 8, ...
           'FontWeight', 'bold', 'EdgeColor', 'none');

% Save the plot
% saveas(gcf, fullfile(output_directory, 'Vehicle_Position_vs_Real_Noise1v6.png'));

saveas(gcf, fullfile(output_directory, 'Vehicle_Cred_and_Trust_Real_Noise1v6.png'));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Normalized Root Mean Squared Error for single scenario

% % Function to compute MSE
% function nrmse_value = compute_nrmse(y_true, y_pred)
%     y_true = y_true(:); % Convert to column vector
%     y_pred = y_pred(:); % Convert to column vector
%     % Compute Mean Squared Error (MSE)
%     mse_value = mean((y_true - y_pred).^2);
%     % Compute Root Mean Squared Error (RMSE)
%     rmse_value = sqrt(mse_value);
%     % Compute Normalized Root Mean Squared Error (NRMSE)
%     nrmse_value = rmse_value / (max(y_true) - min(y_true));
% end
% 
% % Get all vehicle names in the dataset (assuming they are fieldnames of selected_LR_car)
% vehicle_names = fieldnames(selected_LR_car.str_v2v_data);
% num_vehicles = length(vehicle_names);
% 
% % Initialize arrays for NRMSE calculations
% NRMSE_Model_Real = zeros(num_vehicles, 3);  % X, Y, V
% NRMSE_KF_Real = zeros(num_vehicles, 3);     % X, Y, V
% NRMSE_Real_Noisy = zeros(num_vehicles, 3);  % X, Y, V
% NRMSE_KF_Noisy = zeros(num_vehicles, 3);    % X, Y, V
% 
% % Loop through all vehicles
% for j = 1:num_vehicles
%     vehicle_id = vehicle_names{j};
% 
%     % Check if the vehicle exists in all required datasets
%     if isfield(selected_LR_car.str_v2v_data, vehicle_id) && ...
%        isfield(selected_LR_car.model_v2v, vehicle_id) && ...
%        isfield(selected_LR_car.KF_v2v, vehicle_id)
% 
%         % Extract data
%         x_noisy = selected_LR_car.str_v2v_data.(vehicle_id).X;
%         y_noisy = selected_LR_car.str_v2v_data.(vehicle_id).Y;
%         v_noisy = selected_LR_car.str_v2v_data.(vehicle_id).V;
% 
%         x_model = selected_LR_car.model_v2v.(vehicle_id).z(1,:);
%         y_model = selected_LR_car.model_v2v.(vehicle_id).z(2,:);
%         v_model = selected_LR_car.model_v2v.(vehicle_id).z(3,:);
% 
%         x_KF = selected_LR_car.KF_v2v.(vehicle_id).zhatstore(1,:);
%         y_KF = selected_LR_car.KF_v2v.(vehicle_id).zhatstore(2,:);
%         v_KF = selected_LR_car.KF_v2v.(vehicle_id).zhatstore(3,:);
% 
%         x_real = selected_L1_car.str_X;
%         y_real = selected_L1_car.str_Y;
%         v_real = selected_L1_car.str_V;
% 
%         % Compute NRMSE for each category
%         NRMSE_Model_Real(j, :) = [compute_nrmse(x_real, x_model), compute_nrmse(y_real, y_model), compute_nrmse(v_real, v_model)];
%         NRMSE_KF_Real(j, :) = [compute_nrmse(x_real, x_KF), compute_nrmse(y_real, y_KF), compute_nrmse(v_real, v_KF)];
%         NRMSE_Real_Noisy(j, :) = [compute_nrmse(x_real, x_noisy), compute_nrmse(y_real, y_noisy), compute_nrmse(v_real, v_noisy)];
%         NRMSE_KF_Noisy(j, :) = [compute_nrmse(x_KF, x_noisy), compute_nrmse(y_KF, y_noisy), compute_nrmse(v_KF, v_noisy)];
%     else
%         fprintf('Skipping vehicle %s: Missing data.\n', vehicle_id);
%     end
% end
% 
% % Compute AVERAGE NRMSE across all vehicles
% Avg_NRMSE_Model_Real = mean(NRMSE_Model_Real, 1);
% Avg_NRMSE_KF_Real = mean(NRMSE_KF_Real, 1);
% Avg_NRMSE_Real_Noisy = mean(NRMSE_Real_Noisy, 1);
% Avg_NRMSE_KF_Noisy = mean(NRMSE_KF_Noisy, 1);
% 
% % Display results
% fprintf('\n=== AVERAGE NRMSE VALUES ===\n');
% fprintf('Model vs Real:  X=%.4f, Y=%.4f, V=%.4f\n', Avg_NRMSE_Model_Real);
% fprintf('Kalman vs Real: X=%.4f, Y=%.4f, V=%.4f\n', Avg_NRMSE_KF_Real);
% fprintf('Real vs Noisy:  X=%.4f, Y=%.4f, V=%.4f\n', Avg_NRMSE_Real_Noisy);
% fprintf('Kalman vs Noisy:X=%.4f, Y=%.4f, V=%.4f\n', Avg_NRMSE_KF_Noisy);
% 
% % Plot the NRMSE results
% comparison_labels = {'Model vs Real', 'Kalman vs Real', 'Real vs Noisy', 'Kalman vs Noisy'};
% NRMSE_results = [Avg_NRMSE_Model_Real; Avg_NRMSE_KF_Real; Avg_NRMSE_Real_Noisy; Avg_NRMSE_KF_Noisy];
% 
% figure;
% bar(NRMSE_results);
% xticklabels(comparison_labels);
% ylabel('NRMSE');
% title('Average NRMSE for Different Comparisons');
% legend({'X Position', 'Y Position', 'Velocity'}, 'Location', 'best');
% grid on;
% 
% % Save the NRMSE results
% saveas(gcf, fullfile(output_directory, 'NRMSE_Comparison_Across_Vehicles.png'));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc;

%% Normalized Root Mean Squared Error for Multiple Scenarios

% Plot directory
output_directory = 'C:/Users/monea/OneDrive/Documents/MATLAB/I-80-Emeryville-CA/vehicle-trajectory-data/0500pm-0515pm/kalman_filter/data/Picture/';

% Define directory where .mat files are stored
data_directory = 'C:/Users/monea/OneDrive/Documents/MATLAB/I-80-Emeryville-CA/vehicle-trajectory-data/0500pm-0515pm/kalman_filter/data/';

% Define filenames for 1v6 cases (L1 and LR)
files = {
    'L1_Constant_Noise1v6.mat', 'LR_Constant_Noise1v6.mat';
    'L1_No_Noise1v6.mat', 'LR_No_Noise1v6.mat';
    'L1_Real_Noise1v6.mat', 'LR_Real_Noise1v6.mat'
};

% Define scenario labels
scenario_labels = {'(a) Constant Noise', '(b) No Noise', '(c) Real Noise'};

% Define comparison labels
% comparison_labels = {'Model vs Real', 'Kalman vs Real', 'Real vs Noisy', 'Kalman vs Noisy'};
comparison_labels = {'Case (I)', 'Case (II)', 'Case (III)', 'Case (IV)'};

% Initialize array to store NRMSE results for each scenario
% NRMSE_results = zeros(4, 3, 3); % [4 comparisons] x [3 scenarios] x [X, Y, V]
NRMSE_results = zeros(4, 3, 2);

% Function to compute NRMSE
function nrmse_value = compute_nrmse(y_true, y_pred)
    y_true = y_true(:);
    y_pred = y_pred(:);
    mse_value = mean((y_true - y_pred).^2);
    rmse_value = sqrt(mse_value);
    nrmse_value = rmse_value / (max(y_true) - min(y_true));
end

% Process each scenario (Constant Noise, No Noise, Real Noise)
for i = 1:3
    % Load L1 and LR data
    L1_data = load(fullfile(data_directory, files{i, 1}));
    LR_data = load(fullfile(data_directory, files{i, 2}));

    % Select LR and L1 vehicles for comparison
    selected_LR_index = find([L1_data.L1.OgLnID] == 2, 1); % First LR car
    selected_LR_car = L1_data.L1(selected_LR_index);

    selected_L1_index = find([L1_data.L1.ID] == 10000, 1);
    selected_L1_car = L1_data.L1(selected_L1_index);

    % Get vehicle names in LR dataset
    vehicle_names = fieldnames(selected_LR_car.str_v2v_data);
    num_vehicles = length(vehicle_names);

    % Initialize NRMSE storage for this scenario
    % scenario_NRMSE = zeros(num_vehicles, 4, 3); % [vehicles] x [4 comparisons] x [X, Y, V]
    scenario_NRMSE = zeros(num_vehicles, 4, 2);

    % Loop through all vehicles
    for j = 1:num_vehicles
        vehicle_id = vehicle_names{j};

        % Check if the vehicle exists in all required datasets
        if isfield(selected_LR_car.str_v2v_data, vehicle_id) && ...
           isfield(selected_LR_car.model_v2v, vehicle_id) && ...
           isfield(selected_LR_car.KF_v2v, vehicle_id)

            % Extract data
            x_noisy = selected_LR_car.str_v2v_data.(vehicle_id).X;
            % y_noisy = selected_LR_car.str_v2v_data.(vehicle_id).Y;
            v_noisy = selected_LR_car.str_v2v_data.(vehicle_id).V;

            x_model = selected_LR_car.model_v2v.(vehicle_id).z(1,:);
            % y_model = selected_LR_car.model_v2v.(vehicle_id).z(2,:);
            % v_model = selected_LR_car.model_v2v.(vehicle_id).z(3,:);
            v_model = selected_LR_car.model_v2v.(vehicle_id).z(2,:);

            x_KF = selected_LR_car.KF_v2v.(vehicle_id).zhatstore(1,:);
            % y_KF = selected_LR_car.KF_v2v.(vehicle_id).zhatstore(2,:);
            % v_KF = selected_LR_car.KF_v2v.(vehicle_id).zhatstore(3,:);
            v_KF = selected_LR_car.KF_v2v.(vehicle_id).zhatstore(2,:);

            x_real = selected_L1_car.str_X;
            % y_real = selected_L1_car.str_Y;
            v_real = selected_L1_car.str_V;

            % Compute NRMSE for each category (X, Y, V separately)
            % scenario_NRMSE(j, :, :) = [
            %     compute_nrmse(x_real, x_model), compute_nrmse(y_real, y_model), compute_nrmse(v_real, v_model);  % Model vs Real
            %     compute_nrmse(x_real, x_KF), compute_nrmse(y_real, y_KF), compute_nrmse(v_real, v_KF);           % Kalman vs Real
            %     compute_nrmse(x_real, x_noisy), compute_nrmse(y_real, y_noisy), compute_nrmse(v_real, v_noisy);  % Real vs Noisy
            %     compute_nrmse(x_KF, x_noisy), compute_nrmse(y_KF, y_noisy), compute_nrmse(v_KF, v_noisy)         % Kalman vs Noisy
            %     ];
            scenario_NRMSE(j, :, :) = [
                compute_nrmse(x_real, x_model), compute_nrmse(v_real, v_model);  % Model vs Real
                compute_nrmse(x_real, x_KF), compute_nrmse(v_real, v_KF);           % Kalman vs Real
                compute_nrmse(x_real, x_noisy), compute_nrmse(v_real, v_noisy);  % Real vs Noisy
                compute_nrmse(x_KF, x_noisy), compute_nrmse(v_KF, v_noisy)         % Kalman vs Noisy
                ];
        end
    end

    % Store the average NRMSE for this scenario
    NRMSE_results(:, i, :) = squeeze(mean(scenario_NRMSE, 1)); % Average across vehicles
end

% Define colors for X, Y, V
colors = [0.2 0.4 0.8;  % Blue for X
          % 1.0 0.5 0.0;  % Orange for Y
          0.8 0.1 0.1]; % Red for Velocity

% Create figure with 4 subplots (1 row, 4 columns)
fig = figure('Units', 'inches', 'Position', [0, 0, 14, 5]);

% Define bar width (**increase for larger bars**)
bar_width = 2.0; 

for i = 1:3  % 3 scenarios (Constant Noise, No Noise, Real Noise)
    subplot(1, 3, i);
    hold on;

    % Bar graph for X, Y, and Velocity, grouped by comparison
    bar_data = squeeze(NRMSE_results(:, i, :)); % Extract data for this scenario

    % Create grouped bars for X, Y, V
    b = bar(bar_data, 'grouped', 'BarWidth', bar_width);
    % for k = 1:3
    %     b(k).FaceColor = colors(k, :);
    % end
    for k = 1:2
        b(k).FaceColor = colors(k, :);
    end

    xticks(1:4);
    xticklabels(comparison_labels);
    xtickangle(0); % Set labels to **straight horizontal**
    ylabel('NRMSE');
    
    % Increase font size of tick labels
    set(gca, 'FontSize', 11);

    title(scenario_labels{i}, 'FontSize', 12, 'FontWeight', 'bold');
    % legend({'X', 'Y', 'Velocity'}, 'Location', 'best', 'FontSize', 14);
    legend({'X', 'Velocity'}, 'Location', 'best', 'FontSize', 14);
    grid on;
    ylim([0 max(NRMSE_results(:)) * 1.1]); % Adjust y-limit

    % Add a separate legend outside the subplots for case explanations
    annotation('textbox', [0.79, 0.5, 0.2, 0.22], 'String', ...
        {'Case (I) = Model vs Real', ...
        'Case (II) = Kalman vs Real', ...
        'Case (III) = Real vs Noisy', ...
        'Case (IV) = Kalman vs Noisy'}, ...
        'FontSize', 14, 'EdgeColor', 'black', 'LineWidth', 1.5, ...
        'BackgroundColor', 'white');

    hold off;
end

% Save the figure
saveas(fig, fullfile(output_directory, 'NRMSE_Comparison_1v6.png'));



%% Table for computational time
% Define directory where .mat files are stored
data_directory = 'C:/Users/monea/OneDrive/Documents/MATLAB/I-80-Emeryville-CA/vehicle-trajectory-data/0500pm-0515pm/kalman_filter/data/';

% Define filenames for 1v6 cases
files = {
    'Average_Computation_Time_Constant_Noise1v6.mat'
    'Average_Computation_Time_No_Noise1v6.mat'
    'Average_Computation_Time_Real_Noise1v6.mat'
};

% Define labels
labels = {'Constant Noise', 'No Noise', 'Real Noise'};

% Initialize array to store results
avg_times = nan(length(files), 1);

% Load each file and extract avg_time
for i = 1:length(files)
    data = load(fullfile(data_directory, files{i}));
    avg_times(i) = data.avg_time;
end

% Create table for display
T = table(labels', avg_times, 'VariableNames', {'Scenario', 'Avg_Computation_Time (s)'});

% Create figure for displaying the table
fig = figure('Name', 'Computation Time Table', 'NumberTitle', 'off', 'Position', [500, 300, 400, 200]);
uitable('Data', table2cell(T), 'ColumnName', T.Properties.VariableNames, ...
    'RowName', {}, 'Units', 'Normalized', 'Position', [0, 0, 1, 1]);

% Save the figure
saveas(fig, fullfile(output_directory, 'Computation_Times_Table_1v6.png'));

% Display message
disp('Table figure saved as Computation_Times_Table_1v6.png');
