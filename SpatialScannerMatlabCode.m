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

% Cleanup
fclose(f);
disp("File saved: tof.xyz");
disp("Closing: " + s.Port);
delete(s);
clear s;