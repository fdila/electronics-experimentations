data = [];
for i = 1:100
    new_data = testUART(2001, 'uint16');
    data = [data new_data(1:2000)];
end

histogram(data)