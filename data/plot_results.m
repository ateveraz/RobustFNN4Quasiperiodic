clearvars; close all

show = GetLoggedData('settings.json');
data = show.loadBatch();

%%
clc; close all;

figure(1)
subplot(2,1,1)
plot(data.T1.t, data.T1.sa)
grid minor
xlabel('Time [s]', 'Interpreter','latex')
ylabel('Attitude error [rad]', 'Interpreter','latex')
legend({'x', 'y', 'z'}, 'Interpreter','latex')
title('Attitude and position errors')

subplot(2,1,2)
plot(data.T1.t, data.T1.sp)
grid minor
xlabel('Time [s]', 'Interpreter','latex')
ylabel('Position error [m]', 'Interpreter','latex')

figure(2)
plot3(data.T1.x(:,1), data.T1.x(:,2), -data.T1.x(:,3))
grid minor
xlabel('x [m]', 'Interpreter','latex')
ylabel('y [m]', 'Interpreter','latex')
zlabel('z [m]', 'Interpreter','latex')

figure(3)
subplot(2,2,1)
plot(data.T1.t, data.T1.u)
grid minor
xlabel('Time [s]', 'Interpreter','latex')
ylabel('Control input [N]', 'Interpreter','latex')
legend({'x', 'y', 'z'}, 'Interpreter','latex')
title('Control input')

subplot(2,2,2)
plot(data.T1.t, data.T1.fc)
grid minor
xlabel('Time [s]', 'Interpreter','latex')
ylabel('FNN input [N]', 'Interpreter','latex')

subplot(2,2,3)
plot(data.T1.t, data.T1.gamma)
grid minor
xlabel('Time [s]', 'Interpreter','latex')
ylabel('$\hat{\gamma}$', 'Interpreter','latex')

subplot(2,2,4)
plot(data.T1.t, data.T1.monitor)
grid minor
xlabel('Time [s]', 'Interpreter','latex')
ylabel('Monitor', 'Interpreter','latex')