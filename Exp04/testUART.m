function data = testUART(size, type)

function readSerial(src, ~)
    data = read(src,2000,'uint16');
end

device = serialport("COM4",115200);
flush(device);
data = [];

if (strcmp(type,'uint16'))
    bytes = 2*size;
elseif (strcmp(type, 'uint32'))
    bytes = 4*size;
elseif (strcmp(type, 'uint8'))
    bytes = 1*size;
end

configureCallback(device,"byte",4000,@readSerial);
write(device,10,"uint8");

while (numel(data) < size)
    pause(0.01);
end

delete(device);

end

