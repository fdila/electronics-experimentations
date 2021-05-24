function data = readUART(dim, type)
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

if (strcmp(type,'uint16'))
    size = 2;
elseif (strcmp(type, 'uint32'))
    size = 4;
elseif (strcmp(type, 'uint8'))
    size = 1;
end

% Ask MCU to send data
fwrite(obj1, 10, 'uint8');

% Read in non-blocking mode
readasync(obj1, dim*size);

% Do something while the buffer is not full
test = 0;
while (test == 0)
    if(obj1.BytesAvailable == dim*size)
        test = 1;
        stopasync(obj1);
    end
    % Do other stuff
    %
    %
end

data = fread(obj1, dim, type);

fclose(obj1);
end