clear all

global m1 m2 g a b h h2 W L g...
       smax RipForce K1 K2 K3 C x02 y02 z02...
       x1 y1 z1 Boep B1ep B2ep B3ep p1 q1 r1...
       x2 y2 z2 Boec B1ec B2ec B3ec p2 q2 r2...   
       u1 v1 w1 u2 v2 w2...
       GRID Cargo Platform... 
       Jz1 Jx1 Jy1 Jz2 Jx2 Jy2 Ip Ic t...
       count mu Fmax...
       Contact Ground tstop s0 fail timestep...
       ForceTotal pulse peak crash decel crashX crashY crashtype direction...
       offsetx offsety offsetz noMB1 totalweight kevlar chain Kkevlar...
       maxstroke RipForce totalMB1 totalMB2 totalMB3 records...
       total463L totalrestraintweight totallanyardweight check type switches horz
    
Contact = zeros(1,8);
%%%%%%%%%%%%%%% Cargo dimensions for V-22 %%%%%%%%%%%%%%%
g = 9.81; %gravitational constant [m/s^2]
m1 = 70000; %platform (i.e. aircraft) [kg]
% m2 = 18000; %cargo [kg] (40 kips)
% m2 = (30000)*4.4482216/9.81; %cargo [N] (CH-53E)
% m2 = (10000)*4.4482216/9.81;
conv = 0.3048; %to convert from ft to meters

a = 2.2352; %cargo width [m] (88")
b = 2.7432; %cargo length [m] (108")
W = 2.3368; %floor width [m] (92")
L =8.128; %floor length [m] (320")
h = 4*conv;  %Height of restraint attachment on cargo (z-direction) [m]
h2 = .5*conv; %Thickness of platform [m], max height is 2.6 meters

K3 = 1e7; %Spring Constant for interaction between cargo and platform [N/m]
C = 1e7; %"Infinite" Damping
mu = 0; %Coefficient of friction between cargo and platform
kfactor = 1;
kevlar = 0.07315*kfactor; % kevlar weight, kg/m from US Webbing
chain = 3.62873896; %kg for type I steel chain, 9 ft chain = 8 lb
chain2 = 10.88621688; %kg/m for type II steel chain, 9 ft chain = 24 lb
nylon = 3.75 *  0.45359237; %3.75lb to kg, for 5kip CGU/1B nylon strap

%%%%%%%%%%%%%%% MOMENTS OF INERTIA %%%%%%%%%%%%%%%
Jz1 = (m1*(W^2+L^2))/12; %Moment of inertia for platform, rotation about z axis[kg-m2]
Jx1 = (m1*(W^2+h2^2))/12; %Moment of inertia for platform, rotation about x axis[kg-m2]
Jy1 = (m1*(h2^2+L^2))/12; %Moment of inertia for platform, rotation about y axis[kg-m2]
% Jz2 = (m2*(a^2+b^2))/12; %Moment of inertia for cargo, rotation about z axis[kg-m2]
% Jx2 = (m2*(a^2+h^2))/12; %Moment of inertia for cargo, rotation about x axis[kg-m2]
% Jy2 = (m2*(h^2+b^2))/12; %Moment of inertia for cargo, rotation about y axis[kg-m2]
Ip = [Jx1 0 0; 0 Jy1 0; 0 0 Jz1]; %Inertia tensor for platform
Ic = [Jx2 0 0; 0 Jy2 0; 0 0 Jz2]; %Inertia tensor for cargo

%%%\\\\\\\\\\\\\\\\\\\\||||||||||||||||||||||/////////////////%%
%%%||||||||||||||||||||||||| INPUT |||||||| |||||||||||||||||||%% 
%%%////////////////////||||||||||||||||||||||\\\\\\\\\\\\\\\\\%%

% 
%Cargo CG is determined by x2,y2,z2. The Cargo corners have to be
%determined in reference to CG location. If you want CG outside of box, you
%define CG first (x2,y2,z2) and then fill in coordinates for corners 
%Cargo = [Cx  Cy  Cz]  Coordinates of the corners of the Cargo
above = h;
below = 0;
fwd = b/2;
aft = -b/2;
left = a/2;
right = -a/2;
Cargo = [aft right above;...% Cx1 Cy1 Cz1 top of box on Corner #1
         aft right below;...% Cx1 Cy1 Cz2 bottom of box on Corner #1
         aft  left above;...% Cx2 Cy2 Cz1 top of box on Corner #2
         aft  left below;...% Cx2 Cy2 Cz2 bottom of box on Corner #2
         fwd  left above;...% Cx2 Cy2 Cz1 top of box on Corner #3
         fwd  left below;...% Cx2 Cy2 Cz2 bottom of box on Corner #3
         fwd right above;...% Cx2 Cy2 Cz1 top of box on Corner #4
         fwd right below];% Cx2 Cy2 Cz2 bottom of box on Corner #4

%Plaform = [Px Py Pz]  Coordinates of the corners of the Platform       
Platform = [-L/2 -W/2  0;...% Px1 Py1 Pz1 top
            -L/2 -W/2 -h2/2;...% Px1 Py1 Pz2 bottom
            -L/2  W/2  0;...% Px2 py2 Pz1
            -L/2  W/2 -h2/2;...% Px2 Py2 Pz2
             L/2  W/2  0;...% etc
             L/2  W/2 -h2/2;...
             L/2 -W/2  0;...
             L/2 -W/2 -h2/2;...
              0    0  0]; %Last line is CG Location of Platform [x y z 0 0 0 0]]; 

% %%%%%%%%%%%%%% CRASH CHARACTERISTICS %%%%%%%%%%%%%%%
% decel = [1.5 4 8 10 16 1e9];
% direction = 'long'; %long, late, none, vert
% tstop = 0.1; 
% tf = 1.5;
% timestep = 1/10000;
% 
% if direction == 'long'
%     crashtype = 1; %triangle (1), square (2), infinite (3)
%     v0x1 = 13;
%     v0x2 = 13;
%     v0y1 = 0;
%     v0y2 = 0;
%     v0z1 = 0;
%     v0z2 = 0;
%     crashX = decel(1,3)*g;
%     crashY = 0;%magnitude of peak of decel.
%     pulse = v0x2/(0.5*crashX);
% elseif direction == 'late'
%     crashtype = 1; %triangle (1), square (2), infinite (3)
%     v0x1 = 0;
%     v0x2 = 0;
%     v0y1 = 6.4;
%     v0y2 = 6.4;
%     v0z1 = 0;
%     v0z2 = 0;
%     crashX = 0;
%     crashY = decel(1,3)*g;%magnitude of peak of decel.
%     pulse = v0y2/(0.5*crashY);
% elseif direction == 'vert'
%     crashtype = 1; %triangle (1), square (2), infinite (3)
%     v0x1 = 0;
%     v0x2 = 0;
%     v0y1 = 0;
%     v0y2 = 0;
%     v0z1 = 5;
%     v0z2 = 5;
%     crashX = 0;
%     crashY = 0;%magnitude of peak of decel.
%     crashZ = 1.5*g;
%     pulse = v0z2/(0.5*crashZ);
% elseif direction == 'none'
%     crashtype = 1; %triangle (1), square (2), infinite (3)
%     v0x1 = 0;
%     v0x2 = 0;
%     v0y1 = 0;
%     v0y2 = 0;
%     v0z1 = 0;
%     v0z2 = 0;
%     crashX = 0;
%     crashY = 0;%magnitude of peak of decel.
%     pulse = 0;
% elseif direction == 'back'
%     crashtype = 1; %triangle (1), square (2), infinite (3)
%     v0x1 = 6.4;
%     v0x2 = 6.4;
%     v0y1 = 0;
%     v0y2 = 0;
%     v0z1 = 0;
%     v0z2 = 0;
%     crashX = 5*g;
%     crashY = 0;%magnitude of peak of decel.
%     pulse = 0;
% elseif direction == 'huge'
%     v0x1 = 13;
%     v0x2 = 13;
%     v0y1 = 0;
%     v0y2 = 0;
%     v0z1 = 0;
%     v0z2 = 0;
%     crashX = decel(1,end)*g;
%     crashY = 0;%magnitude of peak of decel.
%     crashtype = 3;
%     pulse = 0;
% end
% 
% peak = tstop + pulse/2; %time at peak decel [s]
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


