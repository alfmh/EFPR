EFPR = serialport("COM3", 115200);
configureTerminator(EFPR,"CR/LF");
flush(EFPR);
EFPR.UserData = struct("Data",[],"Count",1)
while(1)
    EFPR.UserData.Count = EFPR.UserData.Count + 1;
    data = readline(EFPR)
    data = str2double(data)
    plot(data)
end