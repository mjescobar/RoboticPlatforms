function [y] = FRECUENCY_PLOT()
x = '../bin/simulation_files/frecuency.txt';
A = importdata(x, '\t');
g = A(:,1);
m = A(:,2);
dst = A(:,3);
th = A(:,4);

figure
hold on

plot(g, m, 'DisplayName', 'Average frecuency');
plot(g, m+dst, 'LineStyle', '--', 'DisplayName', 'Standard desviation');
plot(g, m-dst, 'LineStyle', '--');
plot(g, th, 'Color', [1,0,0], 'DisplayName', 'Threshold');
xlabel('Generation');
ylabel('Frecuency');
legend('show');

end