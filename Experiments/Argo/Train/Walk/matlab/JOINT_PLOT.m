function [y] = JOINT_PLOT(g,p)

Color = [[0,0,0];[0,0,1];[0,1,0];[0,1,1];[1,0,0];[1,0,1];[1,1,0];[0.5,0.5,0.5];[1,0.27,0]];

S0 = '../bin/simulation_files/joints_position/jointsPosition_G';
S1 = '_P';
S2 = '.txt';
x = [S0 num2str(g) S1 num2str(p) S2];

A = importdata(x, '\t');

t = A(:,1);
n = 'Joint ';

figure
hold on

for i = 2 : 10, 
	name = [n num2str(i-2)];
    plot(t, A(:,i), 'Color', Color(i-1,:), 'DisplayName', name)
end

title('Joints Position at time')
xlabel('Time [s]')
ylabel('Joint Position [rad]')
legend('show')




