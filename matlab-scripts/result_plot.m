figure(1);
plot(data_theta(:,1), data_theta(:,2), 'g', data_theta(:,1), data_theta(:,3), 'b')
title('Arm Response Plot');
ylim([-60 60])
ylabel('theta(deg)');
legend('setpoint','simulation','Location','NorthEast')
xlabel('time(s)');
grid on;

figure(2);
plot(data_alpha(:,1), data_alpha(:,2), 'black')
title('Pendulum Response Plot');
ylim([-10 10])
ylabel('alpha(deg)');
xlabel('time(s)');
grid on;

figure(3);
plot(data_Vm(:,1), data_Vm(:,2), 'r')
title('Control Input(V) Response Plot');
ylim([-10 10])
ylabel('u(V)');

xlabel('time(s)');
grid on;

