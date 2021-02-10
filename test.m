clc;clear all;
data = csvread('testing.csv');
p = plot(data(:,3))