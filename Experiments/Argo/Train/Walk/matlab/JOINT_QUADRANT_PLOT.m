function [y] = JOINT_QUADRANT_PLOT(g,p)

Color = [[0,0,0];[0,0,1];[0,1,0];[0,1,1];[1,0,0];[1,0,1];[1,1,0];[0.5,0.5,0.5];[1,0.27,0]];
plot_order = [1 4 7 10 2 5 8 11 3];

S0 = '../bin/simulation_files/joints_position/jointsPosition_G';
S1 = '_P';
S2 = '.txt';
x = [S0 num2str(g) S1 num2str(p) S2];
Title = 'Position of Joint ';

A = importdata(x, '\t');

t = A(:,1);

figure

for i = 2 : 10,
	y = [Title num2str(i-2)];
	subplot(4,3,plot_order(i-1));
	plot(t, A(:,i), 'Color', Color(i-1,:));
	title(y)
	xlabel('Time [s]')
	ylabel('Joint Position [rad]')
end

end

