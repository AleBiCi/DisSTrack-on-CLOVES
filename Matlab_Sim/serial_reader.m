clear all; clc;
%%

port_name = "COM4"; 
baudrate = 115200;

try
    s = serialport(port_name, baudrate);
    configureTerminator(s, "LF");
    
    disp('In ascolto su MATLAB (Premi Ctrl+C per fermare)...');
    
    while true
        if s.NumBytesAvailable > 0
            data = readline(s);
            disp(data);
        end
        pause(0.01); 
    end
catch ME
    disp(ME.message);
end