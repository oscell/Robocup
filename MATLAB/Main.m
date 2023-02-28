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


Positions = strings(1,3);
Positions(1,1) = 'Defender';
Positions(1,2) = 'Goalkeeper';
Positions(1,3) = 'Attacker';




sim = simulation(num_robots,dt,totalTime,num_teams,robot_radius,showEnv,Positions);



% % 
% sim = sim.run();
% 
% sim.summary();

sim.robots.position_class