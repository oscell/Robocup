clear
clc
close all

rng(10)

num_robots = 3;
dt = 1;
totalTime = 20;
num_teams = 2;
robot_radius = 0.15;
showEnv = true;


Positions = {'Goalkeeper','Defender','Attacker'};


sim = simulation(num_robots,dt,totalTime,num_teams,robot_radius,showEnv,Positions);



% % 
% sim = sim.run();
% 
% sim.summary();
sim.show()
sim.robots(2)

xlim([0 11]);   % Without this, axis resizing can slow things down
ylim([0 9]);




