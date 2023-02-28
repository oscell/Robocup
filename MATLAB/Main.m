clear
clc
clf

rng(10)

num_robots = 2;
dt = 0.1;
totalTime = 10;
num_teams = 2;
robot_radius = 0.15;
showEnv = true;

sim = simulation(num_robots,dt,totalTime,num_teams,robot_radius,showEnv);
sim = sim.run();

sim.summary();