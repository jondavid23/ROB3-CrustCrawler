%%%%%%%%%%%
%Inverse dynamics simulation of a 3DOF Robot(CrustCrawler)
%This script is ,ade by Oliver Hansen
%inspired by Shaoping Bai
clear all; close all; clc
%% motion profiles
%=======================  joint 1
A1 = 0.5;%3; % magnitude
f1 = 20;%pi; % frequency

%=======================  joint 2
A2 =0;%0.5; % magnitude
f2 = 0;%3.*pi; % frequency

%=======================  joint 3
A3 = 0;%0.2; % magnitude
f3 = 0;%4.*pi; % frequency


g = 9.801; % Gravitational constant

%% Propperties
%Lenght of link 1
L2=0.2198;
M2=0.8823952e-1;
%Lenght of link 2
L3=0.26428;
M3=.12150869;
%Inertia tensor 1
I1xx=0.6223e-4; I1yx =0; I1zx =0; 
I1xy=0; I1yy=0.5344e-4; I2zy =0.123e-5; 
I1xz=0; I1yz= 0.123e-5; I1zz= 0.3775e-4;
%Inertia tensor 2
I2xx = 0.27038e-3; I2yx= 0.27038e-3; I2zx=0.325e-5;
I2xy=0.2e-7; I2yy=0.26195e-3; I2zy=0.156e-5; 
I2xz= 0.325e-5; I2yz=0.156e-5; I2zz = 0.2254e-4;
%Inertia tensor 3
I3xx=0.28061e-3; I3yx=0; I3xz=-0.122e-5;
I3yx=0; I3yy=0.24023e-3; I3yz=0; 
I3zx= -0.122e-5; I3zy=0; I3zz=0.6925e-4;

%% Simulation properties 
T = 5; % second
N = 500; % resolution
i = 0; 
for t = linspace(0, T, N)
 %thetas 
    % instantaneous time
    i = i + 1; 
    time(i) = t; 
    % Joint 1: angular displacement, velocity, acceleration
    theta1(i) = A1*sin(f1*t);
    thetad1(i) = A1*f1*cos(f1*t); 
    thetadd1(i) = --A1*f1^2*sin(f1*t); 
    
    % Joint 2: angular displacement, velocity, acceleration
    theta2(i) = A2*sin(f2*t);
    thetad2(i) = A2*f2*cos(f2*t); %plus 45degrees
    thetadd2(i) = -A2*f2^2*sin(f2*t); %plus 45degrees
    
    % Joint 3: angular displacement, velocity, acceleration
    theta3(i) = A3*sin(f3*t);
    thetad3(i) = A3*f3*cos(f3*t); %plus 45degrees
    thetadd3(i) = A3*f3^2*sin(f3*t); %plus 45degrees
 %Mass Matrix 
H11 = I1zz+(1/4)*M2*L2^2*cos(theta2(i))^2*sin(theta1(i))^2+(1/4)*M2*L2^2*cos(theta2(i))^2*cos(theta1(i))^2+I2zz+M3*(-L2*cos(theta2(i))*sin(theta1(i))-(1/2)*L3*cos(theta3(i)+theta2(i))*sin(theta1(i)))^2+M3*(L2*cos(theta2(i))*cos(theta1(i))+(1/2)*L3*cos(theta3(i)+theta2(i))*cos(theta1(i)))^2+I3zz;
H12 = -(1/4).*M2.*L2.^2.*sin(theta2(i)).*sin(theta1(i)).^2.*cos(theta2(i))-(1/4).*M2.*L2.^2.*sin(theta2(i)).*cos(theta1(i)).^2.*cos(theta2(i))+(1/2).*cos(theta1(i)).*I2xz+(1/2).*sin(theta1(i)).*I2yz+(1/2).*I2zx.*cos(theta1(i))+(1/2).*I2zy.*sin(theta1(i))+M3.*(L2.*sin(theta2(i)).*sin(theta1(i))+(1/2).*sin(theta1(i)).*L3.*sin(theta3(i))).*(-L2.*cos(theta2(i)).*sin(theta1(i))-(1/2).*L3.*cos(theta3(i)+theta2(i)).*sin(theta1(i)))+M3.*(-L2.*sin(theta2(i)).*cos(theta1(i))-(1/2).*cos(theta1(i)).*L3.*sin(theta3(i))).*(L2.*cos(theta2(i)).*cos(theta1(i))+(1/2).*L3.*cos(theta3(i)+theta2(i)).*cos(theta1(i)))+(1/2).*cos(theta1(i)).*I3xz+(1/2).*sin(theta1(i)).*I3yz+(1/2).*I3zx.*cos(theta1(i))+(1/2).*I3zy.*sin(theta1(i)); 
H13 = (1/2).*M3.*sin(theta1(i)).*L3.*sin(theta3(i)).*(-L2.*cos(theta2(i)).*sin(theta1(i))-(1/2).*L3.*cos(theta3(i)+theta2(i)).*sin(theta1(i)))-(1/2).*M3.*cos(theta1(i)).*L3.*sin(theta3(i)).*(L2.*cos(theta2(i)).*cos(theta1(i))+(1/2).*L3.*cos(theta3(i)+theta2(i)).*cos(theta1(i)))+(1/2).*cos(theta1(i)).*I3xz+(1/2).*sin(theta1(i)).*I3yz+(1/2).*I3zx.*cos(theta1(i))+(1/2).*I3zy.*sin(theta1(i));
H21 = -(1/4).*M2.*L2.^2.*sin(theta2(i)).*sin(theta1(i)).^2.*cos(theta2(i))-(1/4).*M2.*L2.^2.*sin(theta2(i)).*cos(theta1(i)).^2.*cos(theta2(i))+(1/2).*cos(theta1(i)).*I2xz+(1/2).*sin(theta1(i)).*I2yz+(1/2).*I2zx.*cos(theta1(i))+(1/2).*I2zy.*sin(theta1(i))+M3.*(L2.*sin(theta2(i)).*sin(theta1(i))+(1/2).*sin(theta1(i)).*L3.*sin(theta3(i))).*(-L2.*cos(theta2(i)).*sin(theta1(i))-(1/2).*L3.*cos(theta3(i)+theta2(i)).*sin(theta1(i)))+M3.*(-L2.*sin(theta2(i)).*cos(theta1(i))-(1/2).*cos(theta1(i)).*L3.*sin(theta3(i))).*(L2.*cos(theta2(i)).*cos(theta1(i))+(1/2).*L3.*cos(theta3(i)+theta2(i)).*cos(theta1(i)))+(1/2).*cos(theta1(i)).*I3xz+(1/2).*sin(theta1(i)).*I3yz+(1/2).*I3zx.*cos(theta1(i))+(1/2).*I3zy.*sin(theta1(i));
H22 = (1/4).*M2.*L2.^2.*sin(theta2(i)).^2.*sin(theta1(i)).^2+(1/4).*M2.*L2.^2.*sin(theta2(i)).^2.*cos(theta1(i)).^2+cos(theta1(i)).*(I2xx.*cos(theta1(i))+I2yx.*sin(theta1(i)))+sin(theta1(i)).*(I2yx.*cos(theta1(i))+I2yy.*sin(theta1(i)))+M3.*(L2.*sin(theta2(i)).*sin(theta1(i))+(1/2).*sin(theta1(i)).*L3.*sin(theta3(i))).^2+M3.*(-L2.*sin(theta2(i)).*cos(theta1(i))-(1/2).*cos(theta1(i)).*L3.*sin(theta3(i))).^2+cos(theta1(i)).*(I3xx.*cos(theta1(i))+I3yx.*sin(theta1(i)))+sin(theta1(i)).*(I3yx.*cos(theta1(i))+I3yy.*sin(theta1(i)));
H23 = (1/2).*M3.*sin(theta1(i)).*L3.*sin(theta3(i)).*(L2.*sin(theta2(i)).*sin(theta1(i))+(1/2).*sin(theta1(i)).*L3.*sin(theta3(i)))-(1/2).*M3.*cos(theta1(i)).*L3.*sin(theta3(i)).*(-L2.*sin(theta2(i)).*cos(theta1(i))-(1/2).*cos(theta1(i)).*L3.*sin(theta3(i)))+cos(theta1(i)).*(I3xx.*cos(theta1(i))+I3yx.*sin(theta1(i)))+sin(theta1(i)).*(I3yx.*cos(theta1(i))+I3yy.*sin(theta1(i)));
H31 = (1/2).*M3.*sin(theta1(i)).*L3.*sin(theta3(i)).*(-L2.*cos(theta2(i)).*sin(theta1(i))-(1/2).*L3.*cos(theta3(i)+theta2(i)).*sin(theta1(i)))-(1/2).*M3.*cos(theta1(i)).*L3.*sin(theta3(i)).*(L2.*cos(theta2(i)).*cos(theta1(i))+(1/2).*L3.*cos(theta3(i)+theta2(i)).*cos(theta1(i)))+(1/2).*cos(theta1(i)).*I3xz+(1/2).*sin(theta1(i)).*I3yz+(1/2).*I3zx.*cos(theta1(i))+(1/2).*I3zy.*sin(theta1(i));
H32 = (1/2).*M3.*sin(theta1(i)).*L3.*sin(theta3(i)).*(L2.*sin(theta2(i)).*sin(theta1(i))+(1/2).*sin(theta1(i)).*L3.*sin(theta3(i)))-(1/2).*M3.*cos(theta1(i)).*L3.*sin(theta3(i)).*(-L2.*sin(theta2(i)).*cos(theta1(i))-(1/2).*cos(theta1(i)).*L3.*sin(theta3(i)))+cos(theta1(i)).*(I3xx.*cos(theta1(i))+I3yx.*sin(theta1(i)))+sin(theta1(i)).*(I3yx.*cos(theta1(i))+I3yy.*sin(theta1(i)));
H33 = (1/4).*M3.*sin(theta1(i)).^2.*L3.^2.*sin(theta3(i)).^2+(1/4).*M3.*cos(theta1(i)).^2.*L3.^2.*sin(theta3(i)).^2+cos(theta1(i)).*(I3xx.*cos(theta1(i))+I3yx.*sin(theta1(i)))+sin(theta1(i)).*(I3yx.*cos(theta1(i))+I3yy.*sin(theta1(i)));
% %% Velocity vector
 V11 =-thetad3(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) .^ 2 .* sin(theta2(i)) .* cos(theta2(i)) / 0.2e1 + thetad3(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) .* sin(theta2(i)) .^ 2 .* sin(theta3(i)) / 0.2e1 + thetad3(i) .* M3 .* thetad2(i) .* sin(theta1(i)) .^ 2 .* L2 .* sin(theta2(i)) .^ 2 .* L3 .* cos(theta3(i)) / 0.2e1 - thetad3(i) .* M3 .* thetad1(i) .* L3 .* cos(theta1(i)) .^ 2 .* sin(theta3(i)) .* cos(theta2(i)) .^ 2 .* L2 - thetad3(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* sin(theta3(i)) .* cos(theta2(i)) .^ 2 .* cos(theta3(i)) / 0.2e1 + thetad3(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* sin(theta3(i)) .^ 2 .* cos(theta2(i)) .* sin(theta2(i)) / 0.2e1 - thetad3(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) .^ 2 .* sin(theta2(i)) .* cos(theta2(i)) / 0.2e1 + thetad3(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) .* sin(theta2(i)) .^ 2 .* sin(theta3(i)) / 0.2e1 + thetad3(i) .* M3 .* thetad2(i) .* cos(theta1(i)) .^ 2 .* L2 .* sin(theta2(i)) .^ 2 .* L3 .* cos(theta3(i)) / 0.2e1 - thetad3(i) .* M3 .* L3 .* cos(theta3(i)) .* thetad2(i) .* sin(theta1(i)) .^ 2 .* L2 .* cos(theta2(i)) / 0.2e1 - thetad3(i) .* M3 .* L3 .* cos(theta3(i)) .* thetad2(i) .* cos(theta1(i)) .^ 2 .* L2 .* cos(theta2(i)) / 0.2e1 - M3 .* L3 .^ 2 .* cos(theta3(i)) .^ 2 .* thetad3(i) .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta2(i)) / 0.4e1 + M3 .* L3 .^ 2 .* sin(theta3(i)) .^ 2 .* thetad3(i) .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta2(i)) / 0.4e1 + M3 .* L3 .^ 2 .* sin(theta3(i)) .^ 2 .* thetad3(i) .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta2(i)) / 0.4e1 - M3 .* L3 .^ 2 .* cos(theta3(i)) .^ 2 .* thetad3(i) .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta2(i)) / 0.4e1 + M3 .* L3 .^ 2 .* sin(theta3(i)) .^ 2 .* thetad2(i) .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta2(i)) / 0.4e1 + M3 .* L3 .^ 2 .* sin(theta3(i)) .^ 2 .* thetad2(i) .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta2(i)) / 0.4e1 + thetad2(i) .* M3 .* thetad1(i) .* L2 .* sin(theta2(i)) .^ 2 .* sin(theta1(i)) .^ 2 .* L3 .* sin(theta3(i)) - thetad2(i) .* M3 .* thetad1(i) .* L3 .* sin(theta1(i)) .^ 2 .* sin(theta3(i)) .* cos(theta2(i)) .^ 2 .* L2 - thetad2(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* sin(theta3(i)) .* cos(theta2(i)) .^ 2 .* cos(theta3(i)) / 0.2e1 + thetad2(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* sin(theta3(i)) .^ 2 .* cos(theta2(i)) .* sin(theta2(i)) / 0.2e1 - thetad2(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) .^ 2 .* sin(theta2(i)) .* cos(theta2(i)) / 0.2e1 + thetad2(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) .* sin(theta2(i)) .^ 2 .* sin(theta3(i)) / 0.2e1 + thetad2(i) .* M3 .* L3 .* sin(theta3(i)) .* thetad3(i) .* sin(theta1(i)) .^ 2 .* L2 .* sin(theta2(i)) / 0.2e1 + 0.3e1 / 0.4e1 .* thetad2(i) .* M3 .* L3 .^ 2 .* sin(theta3(i)) .* thetad3(i) .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) .* sin(theta2(i)) + thetad2(i) .* M3 .* thetad1(i) .* L2 .* sin(theta2(i)) .^ 2 .* cos(theta1(i)) .^ 2 .* L3 .* sin(theta3(i)) - thetad2(i) .* M3 .* thetad1(i) .* L3 .* cos(theta1(i)) .^ 2 .* sin(theta3(i)) .* cos(theta2(i)) .^ 2 .* L2 - thetad2(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* sin(theta3(i)) .* cos(theta2(i)) .^ 2 .* cos(theta3(i)) / 0.2e1 + thetad2(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* sin(theta3(i)) .^ 2 .* cos(theta2(i)) .* sin(theta2(i)) / 0.2e1 - thetad2(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) .^ 2 .* sin(theta2(i)) .* cos(theta2(i)) / 0.2e1 + thetad2(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) .* sin(theta2(i)) .^ 2 .* sin(theta3(i)) / 0.2e1 + thetad2(i) .* M3 .* L3 .* sin(theta3(i)) .* thetad3(i) .* cos(theta1(i)) .^ 2 .* L2 .* sin(theta2(i)) / 0.2e1 + 0.3e1 / 0.4e1 .* thetad2(i) .* M3 .* L3 .^ 2 .* sin(theta3(i)) .* thetad3(i) .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) .* sin(theta2(i)) - thetad3(i) .* M3 .* thetad1(i) .* L3 .* sin(theta1(i)) .^ 2 .* sin(theta3(i)) .* cos(theta2(i)) .^ 2 .* L2 - thetad3(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* sin(theta3(i)) .* cos(theta2(i)) .^ 2 .* cos(theta3(i)) / 0.2e1 + thetad3(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* sin(theta3(i)) .^ 2 .* cos(theta2(i)) .* sin(theta2(i)) / 0.2e1 + M3 .* thetad2(i) .^ 2 .* sin(theta1(i)) .^ 2 .* L2 .* cos(theta2(i)) .* L3 .* sin(theta3(i)) .* sin(theta2(i)) + M3 .* thetad2(i) .^ 2 .* cos(theta1(i)) .^ 2 .* L2 .* cos(theta2(i)) .* L3 .* sin(theta3(i)) .* sin(theta2(i)) - thetad2(i) .* M2 .* L2 .^ 2 .* cos(theta2(i)) .* sin(theta1(i)) .^ 2 .* thetad1(i) .* sin(theta2(i)) / 0.2e1 - thetad2(i) .* M2 .* L2 .^ 2 .* cos(theta2(i)) .* cos(theta1(i)) .^ 2 .* thetad1(i) .* sin(theta2(i)) / 0.2e1 - 0.2e1 .* thetad2(i) .* M3 .* thetad1(i) .* L2 .^ 2 .* sin(theta2(i)) .* sin(theta1(i)) .^ 2 .* cos(theta2(i)) + thetad2(i) .* M3 .* L3 .^ 2 .* sin(theta3(i)) .^ 2 .* thetad3(i) .* sin(theta1(i)) .^ 2 .* cos(theta2(i)) / 0.2e1 - 0.2e1 .* thetad2(i) .* M3 .* thetad1(i) .* L2 .^ 2 .* sin(theta2(i)) .* cos(theta1(i)) .^ 2 .* cos(theta2(i)) + thetad2(i) .* M3 .* L3 .^ 2 .* sin(theta3(i)) .^ 2 .* thetad3(i) .* cos(theta1(i)) .^ 2 .* cos(theta2(i)) / 0.2e1 - M3 .* thetad2(i) .^ 2 .* sin(theta1(i)) .^ 2 .* L2 .* cos(theta2(i)) .^ 2 .* L3 .* cos(theta3(i)) / 0.2e1 + M3 .* thetad2(i) .^ 2 .* sin(theta1(i)) .^ 2 .* L2 .* sin(theta2(i)) .^ 2 .* L3 .* cos(theta3(i)) / 0.2e1 + M3 .* L3 .* sin(theta3(i)) .* thetad2(i) .^ 2 .* sin(theta1(i)) .^ 2 .* L2 .* sin(theta2(i)) / 0.2e1 + M3 .* L3 .^ 2 .* sin(theta3(i)) .* thetad2(i) .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) .* sin(theta2(i)) / 0.4e1 - M3 .* thetad2(i) .^ 2 .* cos(theta1(i)) .^ 2 .* L2 .* cos(theta2(i)) .^ 2 .* L3 .* cos(theta3(i)) / 0.2e1 + M3 .* thetad2(i) .^ 2 .* cos(theta1(i)) .^ 2 .* L2 .* sin(theta2(i)) .^ 2 .* L3 .* cos(theta3(i)) / 0.2e1 + M3 .* L3 .* sin(theta3(i)) .* thetad2(i) .^ 2 .* cos(theta1(i)) .^ 2 .* L2 .* sin(theta2(i)) / 0.2e1 + M3 .* L3 .^ 2 .* sin(theta3(i)) .* thetad2(i) .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) .* sin(theta2(i)) / 0.4e1 - thetad2(i) .^ 2 .* sin(theta1(i)) .* I2yy .* cos(theta1(i)) + thetad2(i) .^ 2 .* sin(theta1(i)) .* I2xx .* cos(theta1(i)) + thetad2(i) .^ 2 .* sin(theta1(i)) .* I3xx .* cos(theta1(i)) + thetad3(i) .^ 2 .* sin(theta1(i)) .* I3xx .* cos(theta1(i)) - 0.2e1 .* thetad2(i) .* cos(theta1(i)) .^ 2 .* I3yx .* thetad3(i) + 0.2e1 .* thetad2(i) .* sin(theta1(i)) .^ 2 .* I3yx .* thetad3(i) - thetad2(i) .^ 2 .* sin(theta1(i)) .* I3yy .* cos(theta1(i)) - thetad3(i) .^ 2 .* sin(theta1(i)) .* I3yy .* cos(theta1(i)) - M2 .* L2 .^ 2 .* cos(theta2(i)) .^ 2 .* cos(theta1(i)) .^ 2 .* thetad2(i) .^ 2 / 0.4e1 + M2 .* L2 .^ 2 .* sin(theta2(i)) .^ 2 .* cos(theta1(i)) .^ 2 .* thetad2(i) .^ 2 / 0.4e1 - M3 .* thetad2(i) .^ 2 .* sin(theta1(i)) .^ 2 .* L2 .^ 2 .* cos(theta2(i)) .^ 2 + M3 .* thetad2(i) .^ 2 .* sin(theta1(i)) .^ 2 .* L2 .^ 2 .* sin(theta2(i)) .^ 2 - M3 .* thetad2(i) .^ 2 .* cos(theta1(i)) .^ 2 .* L2 .^ 2 .* cos(theta2(i)) .^ 2 + M3 .* thetad2(i) .^ 2 .* cos(theta1(i)) .^ 2 .* L2 .^ 2 .* sin(theta2(i)) .^ 2 - 0.2e1 .* thetad2(i) .* sin(theta1(i)) .* I3yy .* thetad3(i) .* cos(theta1(i)) + 0.2e1 .* thetad2(i) .* sin(theta1(i)) .* I3xx .* thetad3(i) .* cos(theta1(i)) - M2 .* L2 .^ 2 .* cos(theta2(i)) .^ 2 .* sin(theta1(i)) .^ 2 .* thetad2(i) .^ 2 / 0.4e1 + M2 .* L2 .^ 2 .* sin(theta2(i)) .^ 2 .* sin(theta1(i)) .^ 2 .* thetad2(i) .^ 2 / 0.4e1 - thetad3(i) .* M3 .* L3 .^ 2 .* cos(theta3(i)) .^ 2 .* thetad2(i) .* cos(theta1(i)) .^ 2 .* cos(theta2(i)) / 0.4e1 - thetad3(i) .* M3 .* L3 .^ 2 .* cos(theta3(i)) .^ 2 .* thetad2(i) .* sin(theta1(i)) .^ 2 .* cos(theta2(i)) / 0.4e1 + M3 .* L3 .^ 2 .* sin(theta3(i)) .* thetad3(i) .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) .* sin(theta2(i)) / 0.2e1 + M3 .* L3 .^ 2 .* sin(theta3(i)) .* thetad3(i) .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) .* sin(theta2(i)) / 0.2e1 - M3 .* L3 .* cos(theta3(i)) .* thetad3(i) .^ 2 .* sin(theta1(i)) .^ 2 .* L2 .* cos(theta2(i)) / 0.2e1 - M3 .* L3 .* cos(theta3(i)) .* thetad3(i) .^ 2 .* cos(theta1(i)) .^ 2 .* L2 .* cos(theta2(i)) / 0.2e1 + thetad2(i) .^ 2 .* sin(theta1(i)) .^ 2 .* I3yx + thetad3(i) .^ 2 .* sin(theta1(i)) .^ 2 .* I3yx - thetad2(i) .^ 2 .* cos(theta1(i)) .^ 2 .* I2yx - thetad2(i) .^ 2 .* cos(theta1(i)) .^ 2 .* I3yx - thetad3(i) .^ 2 .* cos(theta1(i)) .^ 2 .* I3yx + thetad2(i) .^ 2 .* sin(theta1(i)) .^ 2 .* I2yx - 0.2e1 .* thetad2(i) .* M3 .* thetad1(i) .* L2 .* sin(theta2(i)) .* sin(theta1(i)) .^ 2 .* L3 .* cos(theta3(i)) .* cos(theta2(i)) - 0.2e1 .* thetad2(i) .* M3 .* thetad1(i) .* L2 .* sin(theta2(i)) .* cos(theta1(i)) .^ 2 .* L3 .* cos(theta3(i)) .* cos(theta2(i)) + thetad3(i) .* M3 .* thetad2(i) .* sin(theta1(i)) .^ 2 .* L2 .* cos(theta2(i)) .* L3 .* sin(theta3(i)) .* sin(theta2(i)) / 0.2e1 - thetad3(i) .* M3 .* thetad1(i) .* L2 .* sin(theta2(i)) .* sin(theta1(i)) .^ 2 .* L3 .* cos(theta3(i)) .* cos(theta2(i)) + thetad3(i) .* M3 .* thetad2(i) .* cos(theta1(i)) .^ 2 .* L2 .* cos(theta2(i)) .* L3 .* sin(theta3(i)) .* sin(theta2(i)) / 0.2e1 - thetad3(i) .* M3 .* thetad1(i) .* L2 .* sin(theta2(i)) .* cos(theta1(i)) .^ 2 .* L3 .* cos(theta3(i)) .* cos(theta2(i));
 V21 = M3 .* L3 .^ 2 .* sin(theta3(i)) .* thetad3(i) .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) / 0.2e1 - sin(theta1(i)) .* I2xz .* thetad1(i) .^ 2 / 0.2e1 + cos(theta1(i)) .* I2yz .* thetad1(i) .^ 2 / 0.2e1 - thetad1(i) .^ 2 .* I2zx .* sin(theta1(i)) / 0.2e1 + thetad1(i) .^ 2 .* I2zy .* cos(theta1(i)) / 0.2e1 - sin(theta1(i)) .* I3xz .* thetad1(i) .^ 2 / 0.2e1 + cos(theta1(i)) .* I3yz .* thetad1(i) .^ 2 / 0.2e1 - thetad1(i) .^ 2 .* I3zx .* sin(theta1(i)) / 0.2e1 + thetad1(i) .^ 2 .* I3zy .* cos(theta1(i)) / 0.2e1 + thetad2(i) .* M3 .* L3 .^ 2 .* sin(theta3(i)) .* thetad3(i) .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) / 0.2e1 + thetad2(i) .* M3 .* L3 .^ 2 .* sin(theta3(i)) .* thetad3(i) .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) / 0.2e1 + M3 .* thetad2(i) .^ 2 .* sin(theta1(i)) .^ 2 .* L2 .* cos(theta2(i)) .* L3 .* sin(theta3(i)) / 0.2e1 + M3 .* thetad2(i) .^ 2 .* cos(theta1(i)) .^ 2 .* L2 .* cos(theta2(i)) .* L3 .* sin(theta3(i)) / 0.2e1 + M2 .* L2 .^ 2 .* sin(theta2(i)) .* sin(theta1(i)) .^ 2 .* thetad2(i) .^ 2 .* cos(theta2(i)) / 0.4e1 + M2 .* L2 .^ 2 .* sin(theta2(i)) .* cos(theta1(i)) .^ 2 .* thetad2(i) .^ 2 .* cos(theta2(i)) / 0.4e1 + M3 .* thetad2(i) .^ 2 .* sin(theta1(i)) .^ 2 .* L2 .^ 2 .* cos(theta2(i)) .* sin(theta2(i)) + M3 .* thetad2(i) .^ 2 .* cos(theta1(i)) .^ 2 .* L2 .^ 2 .* cos(theta2(i)) .* sin(theta2(i)) + M2 .* thetad1(i) .^ 2 .* L2 .^ 2 .* cos(theta2(i)) .* sin(theta1(i)) .^ 2 .* sin(theta2(i)) / 0.4e1 + M2 .* thetad1(i) .^ 2 .* L2 .^ 2 .* cos(theta2(i)) .* cos(theta1(i)) .^ 2 .* sin(theta2(i)) / 0.4e1 + M3 .* thetad1(i) .^ 2 .* L2 .^ 2 .* cos(theta2(i)) .* sin(theta1(i)) .^ 2 .* sin(theta2(i)) + M3 .* thetad1(i) .^ 2 .* L2 .^ 2 .* cos(theta2(i)) .* cos(theta1(i)) .^ 2 .* sin(theta2(i)) + thetad3(i) .* M3 .* thetad1(i) .* L3 .* sin(theta1(i)) .^ 2 .* sin(theta3(i)) .* cos(theta2(i)) .* L2 .* sin(theta2(i)) / 0.2e1 + thetad3(i) .* M3 .* thetad1(i) .* L3 .* cos(theta1(i)) .^ 2 .* sin(theta3(i)) .* cos(theta2(i)) .* L2 .* sin(theta2(i)) / 0.2e1 + thetad3(i) .* M3 .* thetad1(i) .* L3 .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) .* sin(theta2(i)) .^ 2 .* L2 / 0.2e1 + thetad3(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) .* sin(theta2(i)) .* sin(theta3(i)) / 0.4e1 + thetad3(i) .* M3 .* thetad1(i) .* L3 .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) .* sin(theta2(i)) .^ 2 .* L2 / 0.2e1 + thetad3(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) .* sin(theta2(i)) .* sin(theta3(i)) / 0.4e1 + thetad3(i) .* M3 .* L3 .* cos(theta3(i)) .* thetad2(i) .* sin(theta1(i)) .^ 2 .* L2 .* sin(theta2(i)) - thetad3(i) .* M3 .* sin(theta1(i)) .^ 2 .* L3 .* cos(theta3(i)) .* thetad1(i) .* L2 .* cos(theta2(i)) / 0.2e1 + thetad3(i) .* M3 .* L3 .* cos(theta3(i)) .* thetad2(i) .* cos(theta1(i)) .^ 2 .* L2 .* sin(theta2(i)) - thetad3(i) .* M3 .* cos(theta1(i)) .^ 2 .* L3 .* cos(theta3(i)) .* thetad1(i) .* L2 .* cos(theta2(i)) / 0.2e1 + M3 .* thetad1(i) .^ 2 .* L2 .* cos(theta2(i)) .* sin(theta1(i)) .^ 2 .* L3 .* cos(theta3(i)) .* sin(theta2(i)) - M3 .* L3 .* sin(theta3(i)) .* thetad3(i) .* sin(theta1(i)) .^ 2 .* thetad1(i) .* L2 .* sin(theta2(i)) / 0.2e1 + M3 .* thetad1(i) .^ 2 .* L2 .* cos(theta2(i)) .* cos(theta1(i)) .^ 2 .* L3 .* cos(theta3(i)) .* sin(theta2(i)) - M3 .* L3 .* sin(theta3(i)) .* thetad3(i) .* cos(theta1(i)) .^ 2 .* thetad1(i) .* L2 .* sin(theta2(i)) / 0.2e1 + M3 .* L3 .^ 2 .* sin(theta3(i)) .* thetad3(i) .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) / 0.2e1 + 0.2e1 .* thetad1(i) .* cos(theta1(i)) .* thetad2(i) .* sin(theta1(i)) .* I3yy + 0.2e1 .* thetad1(i) .* cos(theta1(i)) .* I3yy .* thetad3(i) .* sin(theta1(i)) - 0.2e1 .* thetad1(i) .* thetad2(i) .* cos(theta1(i)) .* I2xx .* sin(theta1(i)) + 0.2e1 .* thetad1(i) .* thetad2(i) .* cos(theta1(i)) .* I2yy .* sin(theta1(i)) - 0.2e1 .* thetad1(i) .* cos(theta1(i)) .* thetad2(i) .* sin(theta1(i)) .* I3xx - 0.2e1 .* thetad1(i) .* cos(theta1(i)) .* I3xx .* thetad3(i) .* sin(theta1(i)) + 0.2e1 .* thetad1(i) .* thetad2(i) .* cos(theta1(i)) .^ 2 .* I3yx + 0.2e1 .* thetad1(i) .* thetad2(i) .* cos(theta1(i)) .^ 2 .* I2yx - 0.2e1 .* thetad1(i) .* thetad2(i) .* sin(theta1(i)) .^ 2 .* I2yx + 0.2e1 .* thetad1(i) .* I3yx .* thetad3(i) .* cos(theta1(i)) .^ 2 - 0.2e1 .* thetad1(i) .* I3yx .* thetad3(i) .* sin(theta1(i)) .^ 2 - 0.2e1 .* thetad1(i) .* thetad2(i) .* sin(theta1(i)) .^ 2 .* I3yx - thetad3(i) .* M3 .* sin(theta1(i)) .^ 2 .* L3 .^ 2 .* cos(theta3(i)) .^ 2 .* thetad1(i) .* cos(theta2(i)) / 0.4e1 - thetad3(i) .* M3 .* cos(theta1(i)) .^ 2 .* L3 .^ 2 .* cos(theta3(i)) .^ 2 .* thetad1(i) .* cos(theta2(i)) / 0.4e1 + M3 .* L3 .* cos(theta3(i)) .* thetad3(i) .^ 2 .* sin(theta1(i)) .^ 2 .* L2 .* sin(theta2(i)) / 0.2e1 + M3 .* L3 .* cos(theta3(i)) .* thetad3(i) .^ 2 .* cos(theta1(i)) .^ 2 .* L2 .* sin(theta2(i)) / 0.2e1 + M3 .* thetad1(i) .^ 2 .* L2 .* cos(theta2(i)) .^ 2 .* sin(theta1(i)) .^ 2 .* L3 .* sin(theta3(i)) / 0.2e1 + M3 .* thetad1(i) .^ 2 .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) .* cos(theta2(i)) .^ 2 .* sin(theta3(i)) / 0.4e1 + M3 .* thetad1(i) .^ 2 .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) .^ 2 .* cos(theta2(i)) .* sin(theta2(i)) / 0.4e1 - M3 .* thetad1(i) .^ 2 .* L3 .* sin(theta1(i)) .^ 2 .* sin(theta3(i)) .* sin(theta2(i)) .^ 2 .* L2 / 0.2e1 - M3 .* thetad1(i) .^ 2 .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* sin(theta3(i)) .^ 2 .* sin(theta2(i)) .* cos(theta2(i)) / 0.4e1 - M3 .* thetad1(i) .^ 2 .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* sin(theta3(i)) .* sin(theta2(i)) .^ 2 .* cos(theta3(i)) / 0.4e1 + M3 .* thetad1(i) .^ 2 .* L2 .* cos(theta2(i)) .^ 2 .* cos(theta1(i)) .^ 2 .* L3 .* sin(theta3(i)) / 0.2e1 + M3 .* thetad1(i) .^ 2 .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) .* cos(theta2(i)) .^ 2 .* sin(theta3(i)) / 0.4e1 + M3 .* thetad1(i) .^ 2 .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) .^ 2 .* cos(theta2(i)) .* sin(theta2(i)) / 0.4e1 - M3 .* thetad1(i) .^ 2 .* L3 .* cos(theta1(i)) .^ 2 .* sin(theta3(i)) .* sin(theta2(i)) .^ 2 .* L2 / 0.2e1 - M3 .* thetad1(i) .^ 2 .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* sin(theta3(i)) .^ 2 .* sin(theta2(i)) .* cos(theta2(i)) / 0.4e1 - M3 .* thetad1(i) .^ 2 .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* sin(theta3(i)) .* sin(theta2(i)) .^ 2 .* cos(theta3(i)) / 0.4e1;
 V31 = thetad2(i) .* M3 .* thetad1(i) .* L2 .* sin(theta2(i)) .* sin(theta1(i)) .^ 2 .* L3 .* sin(theta3(i)) / 0.2e1 - thetad2(i) .* M3 .* thetad1(i) .* L3 .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) .* sin(theta2(i)) .^ 2 .* L2 / 0.2e1 - thetad2(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) .* sin(theta2(i)) .* sin(theta3(i)) / 0.4e1 + thetad2(i) .* M3 .* thetad1(i) .* L2 .* sin(theta2(i)) .* cos(theta1(i)) .^ 2 .* L3 .* sin(theta3(i)) / 0.2e1 - thetad2(i) .* M3 .* thetad1(i) .* L3 .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) .* sin(theta2(i)) .^ 2 .* L2 / 0.2e1 - thetad2(i) .* M3 .* thetad1(i) .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) .* sin(theta2(i)) .* sin(theta3(i)) / 0.4e1 + M3 .* thetad1(i) .* L2 .* cos(theta2(i)) .* sin(theta1(i)) .^ 2 .* L3 .* cos(theta3(i)) .* thetad2(i) / 0.2e1 + M3 .* thetad1(i) .* L2 .* cos(theta2(i)) .* cos(theta1(i)) .^ 2 .* L3 .* cos(theta3(i)) .* thetad2(i) / 0.2e1 - M3 .* thetad2(i) .^ 2 .* cos(theta1(i)) .^ 2 .* L2 .* sin(theta2(i)) .* L3 .* cos(theta3(i)) / 0.2e1 + M3 .* thetad1(i) .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) .^ 2 .* cos(theta2(i)) .* thetad2(i) / 0.4e1 - M3 .* L3 .^ 2 .* sin(theta3(i)) .* thetad2(i) .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) / 0.4e1 - M3 .* L3 .^ 2 .* sin(theta3(i)) .* thetad2(i) .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) / 0.4e1 + M3 .* L3 .^ 2 .* sin(theta3(i)) .* thetad3(i) .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) / 0.4e1 - sin(theta1(i)) .* I3xz .* thetad1(i) .^ 2 / 0.2e1 + cos(theta1(i)) .* I3yz .* thetad1(i) .^ 2 / 0.2e1 - thetad1(i) .^ 2 .* I3zx .* sin(theta1(i)) / 0.2e1 + thetad1(i) .^ 2 .* I3zy .* cos(theta1(i)) / 0.2e1 + M3 .* thetad2(i) .^ 2 .* sin(theta1(i)) .^ 2 .* L2 .* cos(theta2(i)) .* L3 .* sin(theta3(i)) / 0.2e1 + M3 .* thetad2(i) .^ 2 .* cos(theta1(i)) .^ 2 .* L2 .* cos(theta2(i)) .* L3 .* sin(theta3(i)) / 0.2e1 - M3 .* thetad2(i) .^ 2 .* sin(theta1(i)) .^ 2 .* L2 .* sin(theta2(i)) .* L3 .* cos(theta3(i)) / 0.2e1 + M3 .* thetad1(i) .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) .^ 2 .* cos(theta2(i)) .* thetad2(i) / 0.4e1 - thetad2(i) .* M3 .* thetad1(i) .* L3 .* sin(theta1(i)) .^ 2 .* sin(theta3(i)) .* cos(theta2(i)) .* L2 .* sin(theta2(i)) / 0.2e1 - thetad2(i) .* M3 .* thetad1(i) .* L3 .* cos(theta1(i)) .^ 2 .* sin(theta3(i)) .* cos(theta2(i)) .* L2 .* sin(theta2(i)) / 0.2e1 + M3 .* thetad1(i) .^ 2 .* L2 .* cos(theta2(i)) .* sin(theta1(i)) .^ 2 .* L3 .* cos(theta3(i)) .* sin(theta2(i)) / 0.2e1 + M3 .* thetad1(i) .^ 2 .* L2 .* cos(theta2(i)) .* cos(theta1(i)) .^ 2 .* L3 .* cos(theta3(i)) .* sin(theta2(i)) / 0.2e1 + M3 .* L3 .^ 2 .* sin(theta3(i)) .* thetad3(i) .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) / 0.4e1 + 0.2e1 .* thetad1(i) .* cos(theta1(i)) .* thetad2(i) .* sin(theta1(i)) .* I3yy + 0.2e1 .* thetad1(i) .* cos(theta1(i)) .* I3yy .* thetad3(i) .* sin(theta1(i)) - 0.2e1 .* thetad1(i) .* cos(theta1(i)) .* thetad2(i) .* sin(theta1(i)) .* I3xx - 0.2e1 .* thetad1(i) .* cos(theta1(i)) .* I3xx .* thetad3(i) .* sin(theta1(i)) + 0.2e1 .* thetad1(i) .* thetad2(i) .* cos(theta1(i)) .^ 2 .* I3yx + 0.2e1 .* thetad1(i) .* I3yx .* thetad3(i) .* cos(theta1(i)) .^ 2 - 0.2e1 .* thetad1(i) .* I3yx .* thetad3(i) .* sin(theta1(i)) .^ 2 - 0.2e1 .* thetad1(i) .* thetad2(i) .* sin(theta1(i)) .^ 2 .* I3yx + M3 .* thetad1(i) .^ 2 .* L2 .* cos(theta2(i)) .^ 2 .* sin(theta1(i)) .^ 2 .* L3 .* sin(theta3(i)) / 0.2e1 + M3 .* thetad1(i) .^ 2 .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) .* cos(theta2(i)) .^ 2 .* sin(theta3(i)) / 0.4e1 + M3 .* thetad1(i) .^ 2 .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* cos(theta3(i)) .^ 2 .* cos(theta2(i)) .* sin(theta2(i)) / 0.4e1 - M3 .* thetad1(i) .^ 2 .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* sin(theta3(i)) .^ 2 .* sin(theta2(i)) .* cos(theta2(i)) / 0.4e1 - M3 .* thetad1(i) .^ 2 .* L3 .^ 2 .* sin(theta1(i)) .^ 2 .* sin(theta3(i)) .* sin(theta2(i)) .^ 2 .* cos(theta3(i)) / 0.4e1 + M3 .* thetad1(i) .^ 2 .* L2 .* cos(theta2(i)) .^ 2 .* cos(theta1(i)) .^ 2 .* L3 .* sin(theta3(i)) / 0.2e1 + M3 .* thetad1(i) .^ 2 .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) .* cos(theta2(i)) .^ 2 .* sin(theta3(i)) / 0.4e1 + M3 .* thetad1(i) .^ 2 .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* cos(theta3(i)) .^ 2 .* cos(theta2(i)) .* sin(theta2(i)) / 0.4e1 - M3 .* thetad1(i) .^ 2 .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* sin(theta3(i)) .^ 2 .* sin(theta2(i)) .* cos(theta2(i)) / 0.4e1 - M3 .* thetad1(i) .^ 2 .* L3 .^ 2 .* cos(theta1(i)) .^ 2 .* sin(theta3(i)) .* sin(theta2(i)) .^ 2 .* cos(theta3(i)) / 0.4e1;
 V12 =0;
 V13 =0;
 V22 =0;
 V23 =0;
 V32 =0;
V33 =0;
 %% Gravity vector 
 G1 =0;
 G2 =((1/2).*M2.*L2.*cos(theta2(i))+M3.*((1/2).*L3.*cos(theta3(i)+theta2(i))+L2.*cos(theta2(i)))).*g;
 G3 =(1/2).*M3.*g.*L3.*cos(theta3(i)+theta2(i)); 
 %%
  tau_1(i) = H11.*thetadd1(i) + H12.*thetadd2(i) + H13.*thetadd3(i) + ...
         V11.*thetadd1(i) + V12.*thetad2(i) + V13.*thetad3(i) + G1
     tau_2(i) = H21.*theta1(i) + H22.*thetadd2(i) + H23.*thetadd3(i) + ...
         V21.*thetadd1(i) + V22.*thetadd2(i) + V23.*thetad3(i) + G2
     tau_3(i) = H31.*thetadd1(i) + H32.*thetadd2(i) + H33.*thetadd3(i) + ...
       V31.*thetad1(i) + V32.*thetad2(i) + V33.*thetad3(i) + G3;
   %% position 
     %Link 1
     X1(i) = L2.*cos(theta1(i)).*cos(theta2(i));
     Y1(i) = L2.*sin(theta1(i)).*cos(theta2(i));
     Z1(i) = L2.*sin(theta2(i));
     %Link 2
     X2(i) = X1(i) + L3.*cos(theta2(i) + theta3(i)).*cos(theta1(i)); 
     Y2(i) = Y1(i) + L3.*cos(theta2(i) + theta3(i)).*sin(theta1(i));
     Z2(i) = Z1(i) + L3.*sin(theta2(i) + theta3(i));
end 
torque_1 = [time; tau_1]'; 
torque_2 = [time; tau_2]'; 
torque_3 = [time; tau_3]';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot the motion profiles
figure(2)
clf
figure(2)
subplot(3, 1, 1)
hold on
plot(time, theta1, 'b')
plot(time, theta2, 'r')
plot(time, theta3, 'g')
hold off
legend('theta 1', 'theta 2', 'theta 3')
grid on; 
xlabel('time [sec]'); ylabel('angular displacement [rad]'); 
subplot(3, 1, 2)
hold on
plot(time, thetad1, 'b')
plot(time, thetad2, 'r')
plot(time, thetad3, 'g')
hold off
grid on; 
xlabel('time [sec]'); ylabel('angular velocity [rad/s]'); 
subplot(3, 1, 3)
hold on
plot(time, thetadd1, 'b')
plot(time, thetadd2, 'r')
plot(time, thetadd3, 'g')
hold off
grid on; 
xlabel('time [sec]'); ylabel('angular acceleration [rad/s.^2]'); 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot the motor torques
figure(1)
clf
figure(1)
hold on
plot(time, tau_1, 'b')
plot(time, tau_2, 'r')
plot(time, tau_3, 'g')
hold off
legend(' joint 1', 'joint 2', 'joint 3')
grid on; 
xlabel('time [sec]'); ylabel('torques [Nm/rad]'); 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  plot the link
figure(3)
clf
figure(3)
hold on 
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
set(gca,'NextPlot','replaceChildren');
for j = 1 : N
    plot3(0, 0, 0, 'ko', 'MarkerSize', 6, 'Linewidth', 2);
    axis equal; grid on; axis([-1 1 -1 1 -1 1]);
    plot3(X1(j), Y1(j), Z1(j), 'bo', 'MarkerSize', 6, 'Linewidth', 2);
    plot3(X2(j), Y2(j), Z2(j), 'ro', 'MarkerSize', 6, 'Linewidth', 2);
    line([0 X1(j)], [0 Y1(j)], [0 Z1(j)], 'Color', 'b', 'Linewidth', 2); 
    line([X1(j) X2(j)], [Y1(j) Y2(j)], [Z1(j) Z2(j)], 'Color', 'r', 'Linewidth', 2);  
    F(j) = getframe; 
end
hold off
movie(F);
