function data = testUART(size, type)

function readSerial(src, ~)
    data = read(src,size,type);
end

device = serialport("COM4",9600);
flush(device);
data = [];
configureCallback(device,"byte",20,@readSerial);
write(device,10,"uint8");

while (numel(data) < 10)
    pause(0.01);
end

delete(device);

end

