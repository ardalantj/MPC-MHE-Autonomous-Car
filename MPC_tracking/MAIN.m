% Simulation driver file 

clc;
clear all;
close all;


Init();
ref = getTrajectory();

[X,U] = Simulate_Forward(@KinematicModel, @MPC, x0, ref, ts, dt, tf, param);
Visualize();
