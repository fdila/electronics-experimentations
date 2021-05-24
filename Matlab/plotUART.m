function plotUART(data)
    index = data(1001);
    new_data = [data(index:1000), data(1:(index-1))];
    plot(new_data);
end