clear
clc
% close all

rng(10)

%% Simulation time
dt = 0.05;
totalTime = 5;


%% Teams
num_teams = 1;
robot_radius = 0.15;
showEnv = false;
Positions = {'Goalkeeper'};


sim = simulation(dt,totalTime,num_teams,robot_radius,showEnv,Positions);

% figure(1); clf; hold on; grid off; axis([0 11,0 9]); set(gca,'visible','off');
% sim.show(1);


tVec = 0:dt:totalTime;

for idx = 2:numel(tVec)
    %% Update
    sim.ball = sim.ball.update_kick(idx,sim.ball.V,sim.ball.orientation);

    for i = 1:sim.numRobots
        sim.robots(i) = sim.robots(i).update_target(idx,sim.ball.Pose,sim.ball.orientation,sim.ball.V);
    end

    %% Figure
    
    figure(2); clf; hold on; grid off; axis([-5 16,0 20]); %set(gca,'visible','off');
    hold on
    sim.ball.show();
    for i = 1:sim.numRobots
        sim.robots(i).show(idx);
    end
    sim.drawpitch();    
    hold off
end

figure(3); clf; hold on; grid on; axis([0 totalTime,-3 5]);
% disp(tVec(1:idx))
% disp(numel(sim.robots(i).poses(1:idx,1)))
plot(tVec(1:idx),sim.robots(i).poses(1:idx,1))
plot(tVec,sim.robots(i).poses(:,2))
title('x and y movement vs Time')
xlabel('Time')
ylabel('Position (m)')
legend('x-position','y-position')
hold off

figure(4); clf; hold on; grid on; axis([0 totalTime,-1 2]);
plot(tVec,sim.robots(i).vels(:,1))
plot(tVec,sim.robots(i).vels(:,2))
title('Velocity vs Time')
xlabel('Time')
ylabel('Velocity (m/s)')
legend('x-velocity','y-velocity')
hold off

figure(5); clf; hold on; grid on; axis equal;
plot(tVec,sim.robots(i).angles)
title('angle vs Time')
xlabel('Time (s)')
ylabel('Angle (Rad)')
hold off
