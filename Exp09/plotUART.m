function plotUART(data)
    index = data(2001);
    new_data = [data(index:2000), data(1:(index-1))];
    plot(new_data);
end