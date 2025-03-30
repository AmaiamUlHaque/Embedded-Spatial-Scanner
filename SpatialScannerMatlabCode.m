clc;
clear vars;
close all;

% Serial port configuration
s = serialport('COM4', 115200, 'Timeout', 30);
configureTerminator(s, "CR/LF"); % Configure for line endings
disp("Opening: " + s.Port);

flush(s); % Clears input and output buffers
                                                                                                                                                                                                                                    
input("Press Enter to start communication...", 's');
writeline(s, 's');

% Open file for writing
f = fopen("tof.xyz", "w");  
if f == -1
    error("Error opening file.");
end

% Arrays to store coordinates
x_coords = [];
y_coords = [];
z_coords = [];
x_coord = 0;
measurement_count = 0;

% Initialize figure
figure;
hold on;
grid on;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('3D Visualization of ToF Sensor Data');
view(3); % 3D view

measurementsPerRev = 128;
stepAngle = 2.8125;

% Loop to take measurements
try
    for j = 1:10
        for i = 1:measurementsPerRev
            % Read a line from serial
            x = readline(s); 
            
            % Skip empty lines
            if strlength(x) == 0
                warning("Received an empty line. Skipping...");
                continue;
            end
            
            disp("Received: " + x); % Print full line
            
            % Parse the data (expecting: RangeStatus, Distance, SignalRate, AmbientRate, SpadNum)
            data = sscanf(x, '%u, %u, %u, %u, %u');
            
            % Check if we got valid data
            if numel(data) < 2 || data(2) == 0
                warning("Invalid distance data. Skipping...");
                continue;
            end
            
            distance = data(2); % Use the distance measurement
            % Calculate the angle for this iteration (11.25/4° steps)
            angle = stepAngle * (i-1); % 0° starts facing forward (adjust if needed)
            
            % Convert polar to Cartesian coordinates
            y_coord = distance * sind(angle);  % Y coordinate
            z_coord = distance * cosd(angle);  % Z coordinate (up/down)
            
            % Store coordinates
            x_coords(end+1) = x_coord;
            y_coords(end+1) = y_coord;
            z_coords(end+1) = z_coord;
            
            % Write to file
            fprintf(f, "%.4f\t%.4f\t%.4f\n", x_coord, y_coord, z_coord);
            
            % Update plot in real-time
            if measurement_count > 0
                % Connect to previous point
                plot3([x_coords(end-1), x_coords(end)], ...
                      [y_coords(end-1), y_coords(end)], ...
                      [z_coords(end-1), z_coords(end)], ...
                      '-b', 'LineWidth', 1);
            end
            plot3(x_coords(end), y_coords(end), z_coords(end), 'ro', 'MarkerSize', 6);
            
            measurement_count = measurement_count + 1;
        end
        
        % Move to next scan plane
        x_coord = j * 300; % 300mm between scans
    end
    
    % Connect last point to first point to complete the scan
    if ~isempty(x_coords)
        plot3([x_coords(end), x_coords(1)], ...
              [y_coords(end), y_coords(1)], ...
              [z_coords(end), z_coords(1)], ...
              '-b', 'LineWidth', 1);
    end
    
catch ME
    disp("Error occurred: " + ME.message);
end

% Cleanup
fclose(f);
disp("File saved: tof.xyz");
disp("Closing: " + s.Port);
delete(s);
clear s;

% Final plot with all data
figure;
plot3(x_coords, y_coords, z_coords, '-o', ...
      'LineWidth', 1, ...
      'MarkerSize', 6, ...
      'MarkerFaceColor', 'r');
grid on;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('Final 3D Visualization of ToF Sensor Data');
view(3); % 3D view