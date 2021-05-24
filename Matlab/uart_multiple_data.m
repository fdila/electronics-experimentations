data = [];

for i = 1:1000
    new_data = testUART(201, 'uint16');
    data = [data max(new_data(1:200))];
    disp(i);
end

histogram(data,100)