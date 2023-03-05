clear
clc
close all

rng(10)

%% Simulation time
dt = 0.1;
totalTime = 20;

%% Teams
num_teams = 1;
robot_radius = 0.15;
showEnv = false;

%% Ball dynamics


Positions = {'Attacker'};


sim = simulation(dt,totalTime,num_teams,robot_radius,showEnv,Positions);

% figure(1)


% sim = sim.run();
% 
% sim.summary();



tVec = 0:dt:totalTime;

for idx = 2:numel(tVec)
    % Update the environment
    sim = sim.update(idx);
    figure(2); clf; hold on; grid off; axis([0 11,0 9]); set(gca,'visible','off');
    sim.show(idx);
        
end


