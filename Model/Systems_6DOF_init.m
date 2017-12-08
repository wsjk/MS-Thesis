%Run SRDmovie.m to create a movie of the results
clear all

%Global variables passed to the various sub-modules

global m1 m2 g a b h h2 W L g...
       smax RipForce K1 K2 K3 C x02 y02 z02...
       x1 y1 z1 Boep B1ep B2ep B3ep p1 q1 r1...
       x2 y2 z2 Boec B1ec B2ec B3ec p2 q2 r2...   
       u1 v1 w1 u2 v2 w2...
       SRD Cargo Platform... 
       Jz1 Jx1 Jy1 Jz2 Jx2 Jy2 Ip Ic t...
       count mu Fmax...
       Contact TimeOfContact Ground tstop s0 fail timestep...
       ForceTotal pulse peak crash deploy crashX crashY crashtype...
       offsetx offsety offsetz

TimeOfContact = 0;   
Contact = zeros(1,8);
count = 1;
increment = 0;
holden = 1;
%%%%%%%%%%%%%%% Cargo dimensions for V-22 %%%%%%%%%%%%%%%
g = 9.8; %gravitational constant [m/s^2]
m1 = 70000; %platform (i.e. aircraft) [kg]
% m2 = 18000; %cargo [kg] (40 kips)
m2 = (5000)*4.4482216/9.81; %cargo [N] (CH-53E)
conv = 0.3048; %to convert from ft to meters
a = 5*conv;  %Width of cargo (y-direction) [m]
b = 15*conv; %Length of cargo (x-direction) [m]
h = 4*conv;  %Height of SRD attachment on cargo (z-direction) [m]
h2 = .5*conv; %Thickness of platform [m], max height is 2.6 meters
L = 24*conv; %Length of Platform [m]
W = 6*conv; %Width of Platform [m]

while(holden == 1)
increment = increment + 1;
%For those small 200-lb red SRDs we drop tested
%V-138 Poly
%Mil-spec 5625 Nylon
noMB1 = 12+increment; %for 10kip rings
noMB2 = 1;  %for 5 kip rings
maxstroke = 1; %[ft] stroke of SRD
weight = (52*noMB1 + 26*noMB2) * 39.5/1000 * maxstroke; %[lb] weight of an SRD (39.5 grams)
Fmax1 = noMB1 * 52 * 17792.8864; %Failure of SRD = failure of mil-5625 nylon webbing (4000 lb)
Fmax2 = noMB2 * 26 * 17792.8864; %Failure of SRD = failure of mil-5625 nylon webbing (4000 lb)
RipForce1 = 840*52; %Activation force of SRD [N]
RipForce2 = 840*52/2; %Activation force of SRD [N]
% RipForce = 44482.216; %10 kip SRD
E = 50e6; %[Pa] Modulus of nylon webbing 
A = 0.81/100^2; %[m^2]cross section area of webbing (0.81 cm^2)
l1 = 0.0254 * 5.5; %[m] length of SRD wing (5.5 inches)   
l2 = 0.0254 * 22; % [m] length of webbing after full stroke (22 inches)
%K1 = stiffness of 10 kip SRD before activation [N/m], 52 small SRDs for 10 kip
K1 = (E*A/l1)*52; 
%K2 = stiffness of 5 kip SRDs before activation [N/m], 26 small SRDs for 5 kip
K2 = (E*A/l1)*52/2; 
K3 = 1e7; %Spring Constant for interaction between cargo and platform [N/m]
C = 1e7; %"Infinite" Damping
mu = 0; %Coefficient of friction between cargo and platform

%%%%%%%%%%%%%%% MOMENTS OF INERTIA %%%%%%%%%%%%%%%
Jz1 = (m1*(W^2+L^2))/12; %Moment of inertia for platform, rotation about z axis[kg-m2]
Jx1 = (m1*(W^2+h2^2))/12; %Moment of inertia for platform, rotation about x axis[kg-m2]
Jy1 = (m1*(h2^2+L^2))/12; %Moment of inertia for platform, rotation about y axis[kg-m2]
Jz2 = (m2*(a^2+b^2))/12; %Moment of inertia for cargo, rotation about z axis[kg-m2]
Jx2 = (m2*(a^2+h^2))/12; %Moment of inertia for cargo, rotation about x axis[kg-m2]
Jy2 = (m2*(h^2+b^2))/12; %Moment of inertia for cargo, rotation about y axis[kg-m2]
Ip = [Jx1 0 0; 0 Jy1 0; 0 0 Jz1]; %Inertia tensor for platform
Ic = [Jx2 0 0; 0 Jy2 0; 0 0 Jz2]; %Inertia tensor for cargo


%%%%%%%%%%%%%% CRASH CHARACTERISTICS %%%%%%%%%%%%%%%
decel = [1.5 3 6 10 16 21 25 1e9];
direction = 'late'; %long, lat or infinite
crashtype = 1; %triangle (1), square (2), infinite (3)
tstop = 0.1; 
tf = 1.1;
timestep = 1/10000;

if direction == 'long'
    v0x1 = 13;
    v0x2 = 13;
    v0y1 = 0;
    v0y2 = 0;
    crashX = decel(1,3)*g;
    crashY = 0;%magnitude of peak of decel.
%     pulse = v0x2/(0.5*crashX);
pulse = 0.44;
elseif direction == 'late'
    v0x1 = 0;
    v0x2 = 0;
    v0y1 = 6.4;
    v0y2 = 6.4;
    crashX = 0;
    crashY = decel(1,3)*g;%magnitude of peak of decel.
    pulse = v0y2/(0.5*crashY);
end

peak = tstop + pulse/2; %time at peak decel [s]

%%%%%%%%%%%%%%% Initial Conditions for Platform %%%%%%%%%%%%%%%
x01 = 0; %X location of Platform/Aircraft [m]
y01 = 0; %Y location of Platform/Aircraft [m]
z01 = 0; %Z location of Platform/Aircraft [m]
% v0x1 = 0; %X velocity of Platform/Aircraft [m/s]
% v0y1 = 6.4; %Y velocity of Platform/Aircraft [m/s]
v0z1 = 0; %Z velocity of Platform/Aircraft [m/s]
phi01 = 0; %Rotation of Platform about X axis [rad]
theta01 = 0; %Rotation of Platform about Y axis [rad]
psi01 = 0; %Rotation of Platform about Z axis [rad]
p01 = 0; %Angular velocity about X axis of Platform [rad/s]
q01 = 0;%Angular velocity about Y axis of Platform [rad/s]
r01 = 0; %Angular velocity about Z axis of Platform [rad/s]

%%%%%%%%%%%%%% Initial Conditions for Cargo %%%%%%%%%%%%%%%
x02 = 0; %X location of Cargo [m]
y02 = 0; %Y location of Cargo [m]
z02 = 0; %Z location of Cargo [m]
% v0x2 = 0; %X velocity of Cargo [m/s]
% v0y2 = 6.4; %Y velocity of Cargo [m/s]
v0z2 = 0; %Z velocity of Cargo [m/s]
phi02 = 0; %Rotation of Cargo about X axis [rad]
theta02 = 0; %Rotation of Cargo about Y axis [rad]
psi02 = 0; %Rotation of Cargo about Z axis [rad]
p02 = 0; %Angular velocity about X axis of Cargo [rad/s]
q02 = 0; %Angular velocity about Y axis of Cargo [rad/s]
r02 = 0; %Angular velocity about Z axis of Cargo [rad/s]

%%%\\\\\\\\\\\\\\\\\\\\||||||||||||||||||||||/////////////////%%
%%%||||||||||||||||||||||||| INPUT |||||||| |||||||||||||||||||%% 
%%%////////////////////||||||||||||||||||||||\\\\\\\\\\\\\\\\\%%
%SRD =[ fx   fy   fz   cx   cy   cz] initial coordinates
% fx, fy and fz are coordinates of the SRD floor attachment points
% cx, cy and cz are coordinates of the SRD attachment points on cargo
%Positive x is to the right (top/side view)
%Positive y is up (top view)
% Positive z is up from the floor(side/front view)
vert = 30;
horz = 30;
lanyard = 4;
SRDx = b/2 + lanyard*.3048*cos(vert*pi/180)*cos(horz*pi/180);
SRDy = a/2 + lanyard*.3048*cos(vert*pi/180)*sin(horz*pi/180);
SRDz = lanyard*.3048*sin(vert*pi/180);
% SRDx = L/2;
% SRDy = W/2;
% SRDz = 1*conv;
offsetx = 0;
offsety = 0;
offsetz = 1*conv;
Oui = 1;
Non = 0;


% SRD = [
% -10*conv  -SRDy   0  -7.5*conv+offsetx-x02 -a/2-y02  SRDz-z02    K1*noMB1 RipForce1*noMB1 0;... 
% -8*conv   -SRDy   0  -6*conv+offsetx-x02   -a/2-y02  SRDz-z02    K2*noMB2 RipForce2*noMB2 0;...
% -6*conv   -SRDy   0  -4.5*conv+offsetx-x02 -a/2-y02  SRDz-z02    K1*noMB1 RipForce1*noMB1 0;... 
% -4*conv   -SRDy   0  -3*conv+offsetx-x02    -a/2-y02  SRDz-z02    K1*noMB1 RipForce1*noMB1 0;... 
% -2*conv   -SRDy   0  -1.5*conv+offsetx-x02  -a/2-y02  SRDz-z02    K1*noMB1 RipForce1*noMB1 0;... 
%  2*conv   -SRDy   0  2*conv-offsetx-x02    -a/2-y02  SRDz-z02    K1*noMB1 RipForce1*noMB1 0;... 
%  4*conv   -SRDy   0  4*conv-offsetx-x02  -a/2-y02  SRDz-z02    K1*noMB1 RipForce1*noMB1 0;... 
%  6*conv   -SRDy   0  6*conv-offsetx-x02  -a/2-y02  -z02    K1*noMB1 RipForce1*noMB1 0;... 
%  8*conv   -SRDy   0  7*conv-offsetx-x02  -a/2-y02  SRDz-z02    K1*noMB1 RipForce1*noMB1 0;... 
%  10*conv  -SRDy   0  9*conv-offsetx-x02  -a/2-y02  SRDz-z02    K2*noMB2 RipForce2*noMB2 0;... 
% 
% -SRDx  -.5*conv  0 -b/2-x02+offsetx   -.5*conv-y02 1.5*conv-z02  K2*noMB2 RipForce2*noMB2 0;... 
% -10*conv  -.5*conv  0 -b/2-x02+offsetx   -.5*conv-y02 .75*conv-z02  K1*noMB1 RipForce1*noMB1 0;... 
% -8*conv  -.5*conv  0 -b/2-x02+offsetx   -.5*conv-y02 -z02  K1*noMB1 RipForce1*noMB1 0;... 
% -8*conv   .5*conv  0 -b/2-x02+offsetx    .5*conv-y02 -z02  K1*noMB1 RipForce1*noMB1 0;...
% -10*conv   .5*conv  0 -b/2-x02+offsetx    .5*conv-y02 .75*conv-z02  K1*noMB1 RipForce1*noMB1 0;...
% -SRDx      .5*conv  0 -b/2-x02+offsetx    .5*conv-y02 1.5*conv-z02  K2*noMB2 RipForce2*noMB2 0;...
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Midplane between SRDs on
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  the left and right
%  SRDx  -.5*conv  0   b/2-x02+offsetx   -.5*conv-y02 1.5*conv-z02  K2*noMB2 RipForce2*noMB2 0;... 
%  10*conv  -.5*conv  0  b/2-x02+offsetx   -.5*conv-y02 .75*conv-z02  K2*noMB2 RipForce2*noMB2 0;...
%  10*conv   .5*conv  0  b/2-x02+offsetx    .5*conv-y02 .75*conv-z02  K2*noMB2 RipForce2*noMB2 0;...
%  SRDx      .5*conv  0  b/2-x02+offsetx    .5*conv-y02 1.5*conv-z02  K2*noMB2 RipForce2*noMB2 0;...
% 
%  10*conv  SRDy  0   9*conv-offsetx-x02   a/2-y02     SRDz-z02    K2*noMB2 RipForce2*noMB2 0;...  
%  8*conv   SRDy  0   7*conv-offsetx-x02   a/2-y02     SRDz-z02    K1*noMB1 RipForce1*noMB1 0;...
%  6*conv   SRDy  0   6*conv-offsetx-x02   a/2-y02     -z02    K1*noMB1 RipForce1*noMB1 0;...    
%  4*conv   SRDy  0   4*conv-offsetx-x02   a/2-y02     SRDz-z02    K1*noMB1 RipForce1*noMB1 0;... 
%  2*conv   SRDy  0   2*conv-offsetx-x02     a/2-y02     SRDz-z02    K1*noMB1 RipForce1*noMB1 0;... 
% -2*conv   SRDy  0   -1.5*conv+offsetx-x02   a/2-y02     SRDz-z02    K1*noMB1 RipForce1*noMB1 0;... 
% -4*conv   SRDy  0   -3*conv+offsetx-x02     a/2-y02     SRDz-z02    K1*noMB1 RipForce1*noMB1 0;... 
% -6*conv   SRDy  0   -4.5*conv+offsetx-x02  a/2-y02     SRDz-z02    K1*noMB1 RipForce1*noMB1 0;... 
% -8*conv   SRDy  0   -6*conv+offsetx-x02    a/2-y02     SRDz-z02    K2*noMB2 RipForce2*noMB2 0;...
% -10*conv  SRDy  0   -7.5*conv+offsetx-x02  a/2-y02     SRDz-z02    K1*noMB1 RipForce1*noMB1 0;... 
% 
% %outside SRDs
%  -SRDx     SRDy  0  -b/2-x02+offsetx     a/2-y02     SRDz-z02    K2*noMB2  RipForce2*noMB2 1;... 
%  -SRDx    -SRDy  0  -b/2-x02+offsetx    -a/2-y02     SRDz-z02    K2*noMB2  RipForce2*noMB2 1;...
%   SRDx     SRDy  0   b/2-x02+offsetx     a/2-y02     SRDz-z02    K2*noMB2  RipForce2*noMB2 1;... 
%   SRDx    -SRDy  0   b/2-x02+offsetx    -a/2-y02     SRDz-z02    K2*noMB2  RipForce2*noMB2 1];

% 
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
% %outside SRDs
%  -SRDx     SRDy  0  -b/2     a/2     SRDz     K2*noMB2  RipForce2*noMB2 Oui;... 
%  -SRDx    -SRDy  0  -b/2    -a/2     SRDz     K2*noMB2  RipForce2*noMB2 Oui;...
%   SRDx     SRDy  0   b/2     a/2     SRDz     K2*noMB2  RipForce2*noMB2 Oui;... 
%   SRDx    -SRDy  0   b/2    -a/2     SRDz     K2*noMB2  RipForce2*noMB2 Oui];
% % 

SRD = [ 
 -SRDx  SRDy  0  -b/2  a/2  SRDz K1*noMB1 RipForce1*noMB1 Oui;... 
 -SRDx -SRDy  0  -b/2 -a/2  SRDz K1*noMB1 RipForce1*noMB1 Oui;...
  SRDx  SRDy  0   b/2  a/2  SRDz K1*noMB1 RipForce1*noMB1 Oui;...
  SRDx -SRDy  0   b/2 -a/2  SRDz K1*noMB1 RipForce1*noMB1 Oui];


      
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
              
%%%\\\\\\\\\\\\\\\\\\\\||||||||||||||||||||||////////////////////%%%
%%%|||||||||||||||||||||||||| SOLVER ||||||||||||||||||||||||||||%%% 
%%%////////////////////||||||||||||||||||||||\\\\\\\\\\\\\\\\\\\\%%%

%Platform Initial condition vectors
x_01 = [x01; v0x1]; 
y_01 = [y01; v0y1]; 
z_01 = [z01; v0z1]; 
p_01 = [p01];
q_01 = [q01];
r_01 = [r01];
DCM0_pe = Rotation(phi01,theta01,psi01,1); %Rotation from Earth Fixed CS to Platform CS
DCM0_ep = DCM0_pe';
Boep0 = sqrt(0.25 * (1 + trace(DCM0_ep)));
B1ep0 = sqrt(0.25 * (1 + 2*DCM0_ep(1,1) - trace(DCM0_ep)));
B2ep0 = sqrt(0.25 * (1 + 2*DCM0_ep(2,2) - trace(DCM0_ep)));
B3ep0 = sqrt(0.25 * (1 + 2*DCM0_ep(3,3) - trace(DCM0_ep)));
B0ep = [Boep0;B1ep0;B2ep0;B3ep0]; %Euler Parameter Vector, Platform to Earth fixed CS

%Cargo initial conditions vectors
x_02 = [x02; v0x2]; 
y_02 = [y02; v0y2];
z_02 = [z02; v0z2]; 
p_02 = [p02];
q_02 = [q02];
r_02 = [r02];
DCM0_ce = Rotation(phi02,theta02,psi02,1); %Rotation from Earth Fixed to Cargo cs
DCM0_ec = DCM0_ce';
Boec0 = sqrt(0.25 * (1 + trace(DCM0_ec)));
B1ec0 = sqrt(0.25 * (1 + 2*DCM0_ec(1,1) - trace(DCM0_ec)));
B2ec0 = sqrt(0.25 * (1 + 2*DCM0_ec(2,2) - trace(DCM0_ec)));
B3ec0 = sqrt(0.25 * (1 + 2*DCM0_ec(3,3) - trace(DCM0_ec)));
B0ec = [Boec0;B1ec0;B2ec0;B3ec0]; %Euler Parameter Vector, Cargo to Earth fixed CS
[n,m] = size(SRD);
smax = zeros(n,1);
SRDfail = zeros(n,n+1);

for j = 1:n
    if SRD(j,end) == 1
        smax(j,1) = maxstroke*.3048 + (SRD(j,8)/SRD(j,7));
        s0(j,1) = sqrt((SRD(j,4)-SRD(j,1)+x02)^2 + (SRD(j,5)-SRD(j,2)+y02)^2 + (SRD(j,6)-SRD(j,3)+z02)^2);
    end
end %calc smax

%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%
%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%
%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%

t0 = 0;
x0 = [x_01;y_01;z_01;B0ep;p_01;q_01;r_01;x_02;y_02;z_02;B0ec;p_02;q_02;r_02]; %Initial conditions vector
refine = 2;
options = odeset('Events', @events);
time = t0;
xnew = x0.';
teout=[]; %for events fcn
xeout=[]; %for events fcn
ieout=[]; %for events fcn
z = 1;

for p = 1:4
    tspan = [t0:timestep:tf]; %Time interval of integration
    %ode15s for stiff problems
    [t,x,te,xe,ie] = ode45(@SystemSimulation, tspan, x0, options);
    nt = length(t);
    time = [time;t(2:end)];
    xnew = [xnew;x(2:end,:)]; %store results vector
    teout = [teout; te];    % Events at tstart are never reported.
    xeout = [xeout; xe];
    ieout = [ieout; ie];
    e=.5; %Coefficient of restitution
    for i = 1:26
        oldx0(i,1) = x(end,i); %conditions right before or at impact
    end
%     [x0] = Collision(oldx0,e); %new initial conditions after impact
    [x0] = oldx0;

   % A good guess of a valid first time step is the length of 
    % the last valid time step, so use it for faster computation.
%     options = odeset(options,'InitialStep',t(end)-t(1:nt-1));
    t0 = t(end)
    p
    options = odeset(options,'InitialStep',.0001,...
                           'MaxStep',(t(nt)-t(1)));
    if t0 >= tf-timestep
        break;
    end
end %collision stuff

[col,row] = size(xnew);
%Platform results
x1(1:col,1) = xnew(1:end,1); %x location
u1(1:col,1) = xnew(1:end,2); %x velocity
y1(1:col,1) = xnew(1:end,3); %y location
v1(1:col,1) = xnew(1:end,4); %y velocity
z1(1:col,1)= xnew(1:end,5); %z location
w1(1:col,1)= xnew(1:end,6); %z velocity
Boep(1:col,1)= xnew(1:end,7); %Euler parameter
B1ep(1:col,1)= xnew(1:end,8); %Euler parameter 
B2ep(1:col,1)= xnew(1:end,9); %Euler parameter
B3ep(1:col,1)= xnew(1:end,10); %Euler parameter
p1(1:col,1)= xnew(1:end,11); %roll rate
q1(1:col,1)= xnew(1:end,12); %pitch rate
r1(1:col,1)= xnew(1:end,13); %yaw rate
%Cargo Results
x2(1:col,1)= xnew(1:end,14); %x location
u2(1:col,1)= xnew(1:end,15); %x velocity
y2(1:col,1)= xnew(1:end,16); %y location
v2(1:col,1)= xnew(1:end,17); %y velocity
z2(1:col,1)= xnew(1:end,18); %z location
w2(1:col,1)= xnew(1:end,19); %z velocity
Boec(1:col,1)= xnew(1:end,20); %Euler parameter
B1ec(1:col,1)= xnew(1:end,21); %Euler parameter
B2ec(1:col,1)= xnew(1:end,22); %Euler parameter
B3ec(1:col,1)= xnew(1:end,23); %Euler parameter
p2(1:col,1)= xnew(1:end,24); %roll rate
q2(1:col,1)= xnew(1:end,25); %pitch rate
r2(1:col,1)= xnew(1:end,26); %yaw rate

%Intermediate Variables
[i,j] = size(x1);
[n,m] = size(SRD); %n = total number of SRDs 
phi1 = zeros(i,1); %Platform roll attitude
theta1 = zeros(i,1); %Platform roll attitude
psi1 = zeros(i,1);
phi2 = zeros(i,1);
theta2 = zeros(i,1);
psi2 = zeros(i,1);
ForceX = zeros(i,n);
ForceY = zeros(i,n);
ForceZ = zeros(i,n);
ForceTotal = zeros(i,n);
Torques = zeros(i,3);
SumForce = zeros(i,3);
Normal = zeros(i,3);
newstroke = zeros(i,n);
stroke = zeros(i,n);
CargoPointsX = zeros(i,8);
CargoPointsY = zeros(i,8);
CargoPointsZ = zeros(i,8);
Detection = zeros(i,8);
Def = zeros(i,8);

if min(v2-v1) <= 1e-3
    if max(y2-y1) <= .5*conv
          holden = 2;
        break;
    end
end
noMB1
min(v2-v1)
max(y2-y1)

end


for k = 1:n
    if SRD(k,end) == 1
        smax(k,1) = maxstroke*.3048 + (SRD(k,8)/SRD(k,7));
        s0(k,1) = sqrt((SRD(k,4)-SRD(k,1)+x02)^2 + (SRD(k,5)-SRD(k,2)+y02)^2 +...
        (SRD(k,6)-SRD(k,3)+z02)^2);
    end
end%reset smax

for j = 1:i
    [phi1(j,1),theta1(j,1),psi1(j,1)] = DCM2Euler(Boep(j,1),B1ep(j,1),B2ep(j,1),B3ep(j,1));
    [phi2(j,1),theta2(j,1),psi2(j,1)] = DCM2Euler(Boec(j,1),B1ec(j,1),B2ec(j,1),B3ec(j,1));
    
%     [ForceX(j,:), ForceY(j,:), ForceZ(j,:), ForceTotal(j,:),Torques(j,:), SumForce(j,:),...
%     Normal(j,:), CargoPointsX(j,:),CargoPointsY(j,:),CargoPointsZ(j,:), stroke(j,:), newstroke(j,:), Detection(j,:), Def(j,:)] = ForceCalcs(j);
    [ForceX(j,:), ForceY(j,:), ForceZ(j,:), ForceTotal(j,:),Torques(j,:),...
    Normal(j,:), newstroke(j,:)] = ForceCalcs(j);
end%recalculate mid-variables


g=9.8;
sign = 1;
if direction == 'long'
    maxF(sign,1) = crashX/g;
elseif direction == 'late'
    maxF(sign,1) = crashY/g;
end
maxF(sign,2) = 2*noMB1; %number of devices
maxF(sign,3) = RipForce1*noMB1;
maxF(sign,4) = weight*2;
maxF(sign,5) = max(max(newstroke));
if direction == 'long' 
    maxF(sign,6) = max(x2-x1);
    min(u2-u1)
    max(x2-x1)
    disp('longitudinal')
elseif direction == 'late'
    maxF(sign,6) = max(y2-y1);
    min(v2-v1)
    max(y2-y1)
    disp('lateral')
end
maxF(sign,7) = max(max(ForceTotal));
maxF(sign,8) = SRDx;
maxF(sign,9) = SRDy;
maxF(sign,10) = SRDz;
maxF(sign,11) = atan(  (SRD(1,6)) / sqrt( (SRD(1,1)-SRD(1,4))^2 + (SRD(1,2)-SRD(1,5))^2) )*180/pi;
maxF(sign,12) = atan( (SRD(1,2)-SRD(1,5)) / (SRD(1,1)-SRD(1,4)) )*180/pi;

save testing.mat



