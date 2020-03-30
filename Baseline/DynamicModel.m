% This function implements dynamic bicycle model for control and estimation
% design 

clc; clear all; close all;

function [] = vehicleParam()

vehicle.mass = 2300;
vehcile.Iz = 4400; % Moment of inertia of the vehicle 
vehicle.lf = 1.5; % Distance from CoG to front axle in meters 
vehicle.lr = 1.4; % Distance from CoG to rear acle in meters 
vehicle.V = 8 % Vehicle longitudenal speed in m/s

end