data = importdata('kf_output.data');
filtered_x = data(:, 1);
real_x = data(:, 2);
z = data(:, 3);

figure(1);
hold on;
plot(real_x, 'b')
plot(filtered_x, 'r')