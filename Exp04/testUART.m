function data = testUART(size, type)

device = serialport("COM4",9600);
flush(device);
configureCallback(device,"byte",20,@readSerial);
write(device,10,"uint8");

while ( numel(device.UserData) < 10)
    pause(0.01);
end

data = device.UserData;

function readSerial(src, ~)
    if (strcmp(type,'uint16'))
        dim = 2;
    elseif (strcmp(type, 'uint32'))
        dim = 4;
    elseif (strcmp(type, 'uint8'))
        dim = 1;
    end
    data = read(src,size*dim,type);
    src.UserData = data;
    
    disp("ciao");
end

end

