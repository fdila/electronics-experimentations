function testUART()

device = serialport("COM4",9600);
flush(device);
configureCallback(device,"byte",20,@readSerial);
write(device,10,"uint8");

while (true);
end
end

