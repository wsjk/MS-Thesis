
global m1 m2 g a b h h2 W L g...
       smax RipForce K1 K2 K3 C x02 y02 z02...
       x1 y1 z1 Boep B1ep B2ep B3ep p1 q1 r1...
       x2 y2 z2 Boec B1ec B2ec B3ec p2 q2 r2...   
       u1 v1 w1 u2 v2 w2...
       SRD Cargo Platform... 
       Jz1 Jx1 Jy1 Jz2 Jx2 Jy2 Ip Ic t...
       count mu Fmax...
       Contact Ground tstop s0 fail timestep...
       ForceTotal pulse peak crash decel crashX crashY crashtype direction...
       offsetx offsety offsetz noMB1 totalweight kevlar chain Kkevlar...
       maxstroke RipForce1 RipForce2 RipForce3 totalMB1 totalMB2 totalMB3...
       total463L totalSRDweight total_lanyard_weight
   
Contact = zeros(1,8);
%%%%%%%%%%%%%%% Cargo dimensions for V-22 %%%%%%%%%%%%%%%
g = 9.8; %gravitational constant [m/s^2]
m1 = 70000; %platform (i.e. aircraft) [kg]
% m2 = 18000; %cargo [kg] (40 kips)
% m2 = (30000)*4.4482216/9.81; %cargo [N] (CH-53E)
m2 = (5000)*4.4482216/9.81;
conv = 0.3048; %to convert from ft to meters
% a = 5*conv;  %Width of cargo (y-direction) [m]
% b = 15*conv; %Length of cargo (x-direction) [m]
a = (54/12)*conv;  %Width of 463 Pallet (y-direction) [m] 
b = (88/12)*conv; %Length of 463 Pallet (x-direction) [m]
h = 2*conv;  %Height of SRD attachment on cargo (z-direction) [m]
h2 = .5*conv; %Thickness of platform [m], max height is 2.6 meters
L = 24*conv; %Length of Platform [m]
W = 5.5*conv; %Width of Platform [m]
K3 = 1e7; %Spring Constant for interaction between cargo and platform [N/m]
C = 1e7; %"Infinite" Damping
mu = 0; %Coefficient of friction between cargo and platform
kfactor = 1;
kevlar = 0.07315*kfactor; % kevlar weight, kg/m from US Webbing
chain = 3.62873896/(9*0.3048); %kg/m for type I steel chain, 9 ft chain = 8 lb

%For those small 200-lb red SRDs we drop tested
%V-138 Poly
%Mil-spec 5625 Nylon
E = 50e6; %[Pa] Modulus of nylon webbing 
A = 0.81/100^2; %[m^2]cross section area of webbing (0.81 cm^2)
l1 = 0.0254 * 5.5; %[m] length of SRD wing (5.5 inches)   
l2 = 0.0254 * 22; % [m] length of webbing after full stroke (22 inches)

K1 = (E*A/l1)*52;  %K1 = stiffness of 10 kip SRD before activation [N/m], 52 small SRDs for 10 kip

% K1 = 1.3148e+006; %this is for kevlar, the K1 needed to have 90 mm stretch at activation force
% K1 = (E*A/l1)*52*.75; %for 463L pallets, rings are 75% of 10kip rings/SRD
%K2 = stiffness of 5 kip SRDs before activation [N/m], 26 small SRDs for 5 kip

K2 = (E*A/l1)*52/2; 

RipForce1 = 840*52; %Activation force of SRD [N]
% RipForce1 = 840*52*.75; %Activation force of SRD [N], for 463L pallet, rings are only 7,500 lb
RipForce2 = 840*52/2; %Activation force of SRD [N]
RipForce3 = 840*52*1.5; %15kip activation force

maxstroke = 1; %[ft] stroke of SRD

%%%%%%%%%%%%%%% MOMENTS OF INERTIA %%%%%%%%%%%%%%%
Jz1 = (m1*(W^2+L^2))/12; %Moment of inertia for platform, rotation about z axis[kg-m2]
Jx1 = (m1*(W^2+h2^2))/12; %Moment of inertia for platform, rotation about x axis[kg-m2]
Jy1 = (m1*(h2^2+L^2))/12; %Moment of inertia for platform, rotation about y axis[kg-m2]
Jz2 = (m2*(a^2+b^2))/12; %Moment of inertia for cargo, rotation about z axis[kg-m2]
Jx2 = (m2*(a^2+h^2))/12; %Moment of inertia for cargo, rotation about x axis[kg-m2]
Jy2 = (m2*(h^2+b^2))/12; %Moment of inertia for cargo, rotation about y axis[kg-m2]
Ip = [Jx1 0 0; 0 Jy1 0; 0 0 Jz1]; %Inertia tensor for platform
Ic = [Jx2 0 0; 0 Jy2 0; 0 0 Jz2]; %Inertia tensor for cargo

%%%\\\\\\\\\\\\\\\\\\\\||||||||||||||||||||||/////////////////%%
%%%||||||||||||||||||||||||| INPUT |||||||| |||||||||||||||||||%% 
%%%////////////////////||||||||||||||||||||||\\\\\\\\\\\\\\\\\%%
%SRD =[ fx   fy   fz   cx   cy   cz] initial coordinates
% fx, fy and fz are coordinates of the SRD floor attachment points
% cx, cy and cz are coordinates of the SRD attachment points on cargo
%Positive x is to the right (top/side view)
%Positive y is up (top view)
% Positive z is up from the floor(side/front view)

% SRD = [
% -8*conv   -SRDy   0  -7*conv   -a/2   SRDz    K2*noMB2 RipForce2*noMB2 Oui;...
% -6*conv   -SRDy   0  -6*conv   -a/2   SRDz    K1*noMB1 RipForce1*noMB1 Oui;... 
% -4*conv   -SRDy   0  -4*conv   -a/2   SRDz    K1*noMB1 RipForce1*noMB1 Oui;... 
% -2*conv   -SRDy   0  -2*conv   -a/2   SRDz    K1*noMB1 RipForce1*noMB1 Oui;...
%  0*conv   -SRDy   0   0*conv   -a/2   SRDz    K2*noMB2 RipForce2*noMB1 Oui;...
%  2*conv   -SRDy   0   2*conv   -a/2   SRDz    K1*noMB1 RipForce1*noMB1 Oui;... 
%  4*conv   -SRDy   0   4*conv   -a/2   SRDz    K1*noMB1 RipForce1*noMB1 Oui;... 
%  6*conv   -SRDy   0   6*conv   -a/2   SRDz    K1*noMB1 RipForce1*noMB1 Oui;... 
%  8*conv   -SRDy   0   7*conv   -a/2   SRDz    K1*noMB1 RipForce1*noMB1 Oui;... 
% 
% -SRDx     -.5*conv   0  -b/2  -.5*conv  1.5*conv    K2*noMB2 RipForce2*noMB2 Oui;... 
% -10*conv  -.5*conv   0  -b/2  -.5*conv  0.75*conv   K1*noMB1 RipForce1*noMB1 Oui;... 
% -8*conv   -.5*conv   0  -b/2  -.5*conv  0           K1*noMB1 RipForce1*noMB1 Oui;... 
% -8*conv    .5*conv   0  -b/2   .5*conv  0           K1*noMB1 RipForce1*noMB1 Oui;...
% -10*conv   .5*conv   0  -b/2   .5*conv  0.75*conv   K1*noMB1 RipForce1*noMB1 Oui;...
% -SRDx      .5*conv   0  -b/2   .5*conv  1.5*conv    K2*noMB2 RipForce2*noMB2 Oui;...
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Midplane between SRDs on
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  the left and right
%  SRDx    -.5*conv    0   b/2  -.5*conv  1.5*conv    K2*noMB2 RipForce2*noMB2 Oui;... 
%  10*conv -.5*conv    0   b/2  -.5*conv  0.75*conv   K2*noMB2 RipForce2*noMB2 Oui;...
%  10*conv  .5*conv    0   b/2   .5*conv  0.75*conv   K2*noMB2 RipForce2*noMB2 Oui;...
%  SRDx     .5*conv    0   b/2   .5*conv  1.5*conv    K2*noMB2 RipForce2*noMB2 Oui;...
% 
%  8*conv   SRDy  0   7*conv   a/2     SRDz     K1*noMB1 RipForce1*noMB1 Oui;...
%  6*conv   SRDy  0   6*conv   a/2     SRDz     K1*noMB1 RipForce1*noMB1 Oui;...    
%  4*conv   SRDy  0   4*conv   a/2     SRDz     K1*noMB1 RipForce1*noMB1 Oui;... 
%  2*conv   SRDy  0   2*conv   a/2     SRDz     K1*noMB1 RipForce1*noMB1 Oui;... 
%  0*conv   SRDy  0   0*conv   a/2     SRDz     K2*noMB2 RipForce2*noMB1 Oui;...
% -2*conv   SRDy  0  -2*conv   a/2     SRDz     K1*noMB1 RipForce1*noMB1 Oui;... 
% -4*conv   SRDy  0  -4*conv   a/2     SRDz     K1*noMB1 RipForce1*noMB1 Oui;... 
% -6*conv   SRDy  0  -6*conv   a/2     SRDz     K1*noMB1 RipForce1*noMB1 Oui;... 
% -8*conv   SRDy  0  -7*conv   a/2     SRDz     K2*noMB2 RipForce2*noMB2 Oui;...
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

%%%%%%%%%%%%%% CRASH CHARACTERISTICS %%%%%%%%%%%%%%%
decel = [1.5 3 6 10 16 21 25 1e9];
% direction = 'long'; %long, late, none, vert
crashtype = 1; %triangle (1), square (2), infinite (3)
tstop = 0.1; 
tf = .5;
timestep = 1/10000;

if direction == 'long'
    v0x1 = 13;
    v0x2 = 13;
    v0y1 = 0;
    v0y2 = 0;
    v0z1 = 0;
    v0z2 = 0;
    crashX = decel(1,5)*g;
    crashY = 0;%magnitude of peak of decel.
    pulse = v0x2/(0.5*crashX);
elseif direction == 'late'
    v0x1 = 0;
    v0x2 = 0;
    v0y1 = 6.4;
    v0y2 = 6.4;
    v0z1 = 0;
    v0z2 = 0;
    crashX = 0;
    crashY = decel(1,4)*g;%magnitude of peak of decel.
    pulse = v0y2/(0.5*crashY);
elseif direction == 'vert'
    v0x1 = 0;
    v0x2 = 0;
    v0y1 = 0;
    v0y2 = 0;
    v0z1 = 5;
    v0z2 = 5;
    crashX = 0;
    crashY = 0;%magnitude of peak of decel.
    crashZ = 1.5*g;
    pulse = v0z2/(0.5*crashZ);
elseif direction == 'none'
    v0x1 = 0;
    v0x2 = 0;
    v0y1 = 0;
    v0y2 = 0;
    v0z1 = 0;
    v0z2 = 0;
    crashX = 0;
    crashY = 0;%magnitude of peak of decel.
    pulse = 0;
end

peak = tstop + pulse/2; %time at peak decel [s]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    