function data = readSerial(src,evt)
    data = read(src,10,"uint16");
    disp(data);
end