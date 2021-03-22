function data = readUART()
% Instrument Connection

% Find a serial port object.
obj1 = instrfind('Type', 'serial', 'Port', 'COM4', 'Tag', '');

% Create the serial port object if it does not exist
% otherwise use the object that was found.
if isempty(obj1)
    obj1 = serial('COM4');
else
    fclose(obj1);
    obj1 = obj1(1);
end

% Connect to instrument object, obj1.
fopen(obj1);

flushinput(obj1);

% Ask MCU to send data
fwrite(obj1, 10, 'uint8');

readasync(obj1, 20);

test = 0;
while (test == 0)
    if(obj1.BytesAvailable == 20)
        test = 1;
    end
end

data = fread(obj1, 10, 'uint16');

fclose(obj1);
end