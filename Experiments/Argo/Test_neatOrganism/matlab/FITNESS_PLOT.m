function [y] = FITNESS_PLOT()
x = '../bin/simulation_files/fitness.txt';
A = importdata(x, '\t');
g = A(:,1);
m = A(:,2);
dst = A(:,3);

figure
hold on

plot(g, m, 'DisplayName', 'Average fitness')
plot(g, m+dst, 'LineStyle', '--', 'DisplayName', 'Standard desviation')
plot(g, m-dst, 'LineStyle', '--')
xlabel('Generation')
ylabel('Fitness')
legend('show')
end

