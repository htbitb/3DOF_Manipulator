%% Authors:
% Made by Tran Duc Thien
% Date: 2021/08/31:
% Lagrange approach
% clc;
% clear all;
% close all;
syms e1 e2 e3 L1 L2 L3 de1 de2 de3 m1 m2 m3 I1 I2 I3 g dde1 dde2 dde3 x1 x2 y1 y2 z1 z2
assume(L1,'real');
assume(L2,'real');
assume(L3,'real');
assume(e1,'real');
assume(e2,'real');
assume(e3,'real');
assume(de1,'real');
assume(de2,'real');
assume(de3,'real');
assume(m1,'real');
assume(m2,'real');
assume(m3,'real');
assume(I1,'real');
assume(I2,'real');
assume(I3,'real');
assume(g,'real');
assume(dde1,'real');
assume(dde2,'real');
assume(dde3,'real');
assume(x1,'real');
assume(x2,'real');
assume(y1,'real');
assume(y2,'real');
assume(z1,'real');
assume(z2,'real');
%% Forward kinematics:
[T01,R01,R10,P01] = FKRobot2021( 0,0,0,e1 );
[T12,R12,R21,P12] = FKRobot2021( 0,L1,0,e2 );
%% Position of center of mass in each linkage:
P1c=[x1 y1 z1]';
P2c=[x2 y2 z2]';
w0=[0 0 0]';
v0=[0 0 0]';
%% linkage 1:
% Center of mass position of the first link in frame 0.
TP01c=(T01*[P1c; 1]);
P01c=TP01c(1:3);
% Angular velocity in frame 1:
w1=R10*w0+de1*[0 0 1]';
% Linear velocity of the origin in frame 1:
v1=R10*(v0+[0 -w0(3) w0(2);w0(3) 0 -w0(1);-w0(2) w0(1) 0]*P01);
% Linear velocity of the center of mass in frame 0:
v01c=R01*(v1+[0 -w1(3) w1(2);w1(3) 0 -w1(1);-w1(2) w1(1) 0]*P1c);
%% Linkage 2:
% Center of mass position of the second link in frame 0.
TP02c=(T01*T12*[P2c; 1]);
P02c=TP02c(1:3);
% Angular velocity in frame 2:
w2=R21*w1+de2*[0 0 1]';
% Linear velocity of the origin in frame 2:
v2=R21*(v1+[0 -w1(3) w1(2);w1(3) 0 -w1(1);-w1(2) w1(1) 0]*P12);
% Linear velocity of the center of mass in frame 0:
v02c=R01*R12*(v2+[0 -w2(3) w2(2);w2(3) 0 -w2(1);-w2(2) w2(1) 0]*P2c);
 %% Linkage 3:
 % Center of mass position of the second link in frame 0.
 TP03c=(T01*T12*T23*[P3c; 1]);
 P03c=TP03c(1:3);
 % Angular velocity in frame 2:
 w3=R32*w2+de3*[0 0 1]';
 % Linear velocity of the origin in frame 2:
 v3=R32*(v2+[0 -w2(3) w2(2);w2(3) 0 -w2(1);-w2(2) w2(1) 0]*P23);
 % Linear velocity of the center of mass in frame 0:
 v03c=R01*R12*R23*(v3+[0 -w3(3) w3(2);w3(3) 0 -w3(1);-w3(2) w3(1) 0]*P3c);