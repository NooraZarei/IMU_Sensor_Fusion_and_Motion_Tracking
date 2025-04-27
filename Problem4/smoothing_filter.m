% Load data from CSV file
filename = 'Raw_Angles.csv'; 
data = readtable(filename);

% Extract data columns
timeframes = data.id; 
thetax_noisy = data.theta_x; 
thetay_noisy = data.theta_y; 

% Smooth thetax and thetay using the Savitzky-Golay filter
window_size = 11; % Window size 
order = 3; % Polynomial order
thetax_smooth = sgolayfilt(thetax_noisy, order, window_size);
thetay_smooth = sgolayfilt(thetay_noisy, order, window_size);

smoothed_data = table(timeframes, thetax_smooth, thetay_smooth);
output_filename = 'Smooth_Angles.csv';
writetable(smoothed_data, output_filename);

% Plot original and smoothed data
figure;
plot(timeframes, thetax_noisy, 'b', 'LineWidth', 1.5);
hold on;
plot(timeframes, thetay_noisy, 'r', 'LineWidth', 1.5);
plot(timeframes, thetax_smooth, 'g', 'LineWidth', 1.5);
plot(timeframes, thetay_smooth, 'm', 'LineWidth', 1.5);
xlabel('Time');
ylabel('Angle');
title('Noisy and Smoothed Data');
legend('Noisy thetax', 'Noisy thetay', 'Smoothed thetax', 'Smoothed thetay');
grid on;
