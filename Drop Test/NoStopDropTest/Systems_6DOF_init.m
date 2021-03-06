%Run SRDmovie.m to create a movie of the results
clear all

%Global variables passed to the various sub-modules

global m1 m2 g a b h h2 W L g...
       smax RipForce K1 K2 K3 C...
       x1 y1 z1 Boep B1ep B2ep B3ep p1 q1 r1...
       x2 y2 z2 Boec B1ec B2ec B3ec p2 q2 r2...   
       u1 v1 w1 u2 v2 w2...
       SRD Cargo Platform... 
       Jz1 Jx1 Jy1 Jz2 Jx2 Jy2 Ip Ic...
       count mu Fmax...
       Contact TimeOfContact Ground tstop broken timestep s0

TimeOfContact = 0;   
Contact = zeros(1,8);
count = 1;

g = 9.8; %gravitational constant [m/s^2]
m1 = 70000; %platform (i.e. aircraft) [kg]
a = 8; %Width of cargo (y-direction) [m]
b = 8; %Length of cargo (x-direction) [m]
h = 1;  %Height of SRD attachment on cargo (z-direction) [m]
h2 = .25; %Thickness of platform which the cargo is on top of [m]
W = 20; %Width of Platform (Aircraft) [m]
L = 20; %Length of Platform [m]

m2 = 0.45359237 * 65;

%For those small 200-lb red SRDs we drop tested
%V-138 Polyester
%Mil-spec 5625 Nylon
noSRD = 0;
maxstroke = 1; %[ft] stroke of SRD
smax = maxstroke*.3048;
weight = 2 * noSRD * 0.0022046226218 * 39.5 %[lb] weight of an SRD (39.5 grams)
Fmax = noSRD * 17792.8864; %Failure of SRD = failure of mil-5625 nylon webbing (4000 lb)
RipForce = 840*noSRD; %Activation force of SRD [N]
E = 50e6; %[Pa] Modulus of nylon webbing 
A = 0.81/100^2; %[m^2]cross section area of webbing (0.81 cm^2)
l = 0.0254 * 5.5; %[m] length of SRD wing (5.5 inches)   
K1 = (E*A/l)*noSRD; %Spring constant of SRD before activation [N/m]
K2 = K1; %Spring constant of SRD after fully stroked [N/m]
drop = 0.5 + 0.0254*30

K3 = 2e9; %Spring Constant for interaction between cargo and platform [N/m]
C = 1e9; %"Infinite" Damping
mu = 0; %Coefficient of friction between cargo and platform

Jz1 = (m1*(W^2+L^2))/12; %Moment of inertia for platform, rotation about z axis[kg-m2]
Jx1 = (m1*(W^2+h2^2))/12; %Moment of inertia for platform, rotation about x axis[kg-m2]
Jy1 = (m1*(h2^2+L^2))/12; %Moment of inertia for platform, rotation about y axis[kg-m2]
Jz2 = (m2*(a^2+b^2))/12; %Moment of inertia for cargo, rotation about z axis[kg-m2]
Jx2 = (m2*(a^2+h^2))/12; %Moment of inertia for cargo, rotation about x axis[kg-m2]
Jy2 = (m2*(h^2+b^2))/12; %Moment of inertia for cargo, rotation about y axis[kg-m2]
Ip = [Jx1 0 0; 0 Jy1 0; 0 0 Jz1]; %Inertia tensor for platform
Ic = [Jx2 0 0; 0 Jy2 0; 0 0 Jz2]; %Inertia tensor for cargo

%Initial Conditions for Platform
x01 = 0; %X location of Platform/Aircraft [m]
y01 = 0; %Y location of Platform/Aircraft [m]
z01 = 5; %Z location of Platform/Aircraft [m]
v0x1 = 0; %X velocity of Platform/Aircraft [m/s]
v0y1 = 0; %Y velocity of Platform/Aircraft [m/s]
v0z1 = 0; %Z velocity of Platform/Aircraft [m/s]
phi01 = 0; %Rotation of Platform about X axis [rad]
theta01 = pi; %Rotation of Platform about Y axis [rad]
psi01 = 0; %Rotation of Platform about Z axis [rad]
p01 = 0; %Angular velocity about X axis of Platform [rad/s]
q01 = 0;%Angular velocity about Y axis of Platform [rad/s]
r01 = 0; %Angular velocity about Z axis of Platform [rad/s]

%Initial Conditions for Cargo
x02 = 0; %X location of Cargo [m]
y02 = 0; %Y location of Cargo [m]
z02 = 4.5; %Z location of Cargo [m]
v0x2 = 0; %X velocity of Cargo [m/s]
v0y2 = 0; %Y velocity of Cargo [m/s]
v0z2 = 0; %Z velocity of Cargo [m/s]
phi02 = 0; %Rotation of Cargo about X axis [rad]
theta02 = pi; %Rotation of Cargo about Y axis [rad]
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

SRDx = 4+1.5;
SRDy = 4+sqrt(3)/2;
% SRD = [ -SRDx   -SRDy  0   -4   -4   h;... %SRD1 (bottom left corner) 
%         -SRDx    SRDy  0   -4    4   h;... %SRD2 (top left corner)
%        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Midplane between SRDs on
%        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  the left and right
%          SRDx    SRDy  0    4    4   h;... %SRD3 (top right corner)  
%          SRDx   -SRDy  0    4   -4   h] %SRD4 (bottom right corner)
     
% SRD = [ -SRDx   -SRDy  0   -4   -4   h;... %SRD1 (bottom left corner) 
%         -SRDx    SRDy  0   -4    4   h] %SRD4 (bottom right corner)
%      
SRD = [0   0   0   0   0  h/2];
     

% SRD = [ -SRDx  -SRDy   0  -4   -4   h;... %SRD1 (bottom left corner) 
%         -SRDx   SRDy   0  -4    4   h;... %SRD2 (top left corner)
%         -8.5   -SRDy   0  -4   -1   h/2;...
%         -8.5    SRDy   0  -4    1   h/2;...
%         -6.5   -SRDy   0  -4   -2   h;...
%         -6.5    SRDy   0  -4    2   h;...
%        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Midplane between SRDs on
%        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  the left and right
%          6.5   -SRDy   0   4   -2   h;...
%          6.5    SRDy   0   4    2   h;...
%          8.5   -SRDy   0   4   -1   h/2;...
%          8.5    SRDy   0   4    1   h/2;...  
%          SRDx   SRDy   0   4    4   h;... %SRD3 (top right corner)  
%          SRDx  -SRDy   0   4   -4   h] %SRD4 (bottom right corner).
     
% SRD = [ -6   -6   0  -4   -4   h;... %SRD1 (bottom left corner) 
%         -6    6   0  -4    4   h] %SRD2 (top left corner)

%Cargo = [Cx  Cy  Cz]  Coordinates of the corners of the Cargo
Cargo = [-b/2 -a/2 h;...% Cx1 Cy1 Cz1 top of box on Corner #1
         -b/2 -a/2 0;...% Cx1 Cy1 Cz2 bottom of box on Corner #1
         -b/2  a/2 h;...% Cx2 Cy2 Cz1 top of box on Corner #2
         -b/2  a/2 0;...% Cx2 Cy2 Cz2 bottom of box on Corner #2
          b/2  a/2 h;...% Cx2 Cy2 Cz1 top of box on Corner #3
          b/2  a/2 0;...% Cx2 Cy2 Cz2 bottom of box on Corner #3
          b/2 -a/2 h;...% Cx2 Cy2 Cz1 top of box on Corner #4
          b/2 -a/2 0;...% Cx2 Cy2 Cz2 bottom of box on Corner #4
           0    0  0]; %Last line is CG Location of cargo [x y z 0 0 0 0]]; 

%Plaform = [Px Py Pz]  Coordinates of the corners of the Platform       
Platform = [-L/2 -W/2  0;...% Px1 Py1 Pz1 top
            -L/2 -W/2 -h/2;...% Px1 Py1 Pz2 Lottom
            -L/2  W/2  0;...% Px2 py2 Pz1
            -L/2  W/2 -h/2;...% Px2 Py2 Pz2
             L/2  W/2  0;...% etc
             L/2  W/2 -h/2;...
             L/2 -W/2  0;...
             L/2 -W/2 -h/2;...
              0    0  0]; %Last line is CG Location of Platform [x y z 0 0 0 0]]; 
          %Plaform = [Px Py Pz]  Coordinates of the corners of the Platform       
% Ground = [  -100 -100 -h/2;...% Px1 Py1 Pz1 top
%             -100 -100 -h/2;...% Px1 Py1 Pz2 Lottom
%             -100  100 -h/2;...% Px2 py2 Pz1
%             -100  100 -h/2;...% Px2 Py2 Pz2
%              100  100 -h/2;...% etc
%              100  100 -h/2;...
%              100 -100 -h/2;...
%              100 -100 -h/2;...
%               0    0  0]; %Last line is CG Location of Platform [x y z 0 0 0 0]]; 
                
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
broken = zeros(n,1);
j = 1;
s0 = sqrt((SRD(j,4)-SRD(j,1))^2 + (SRD(j,5)-SRD(j,2))^2 + (SRD(j,6)-SRD(j,3))^2)+drop; 
%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%
%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%
%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%

t0 = 0;
timestep = 1/10000;
tf = 1;
tstop = 0.1; %Time when platform hits wall. NOTE: if tstop = 0, then platform does not hit wall
x0 = [x_01;y_01;z_01;B0ep;p_01;q_01;r_01;x_02;y_02;z_02;B0ec;p_02;q_02;r_02]; %Initial conditions vector
refine = 2;
options = odeset('Events', @events);
time = t0;
xnew = x0.';
teout=[]; %for events fcn
xeout=[]; %for events fcn
ieout=[]; %for events fcn
z = 1;
for p = 1:5
    tspan = [t0:timestep:tf]; %Time interval of integration
    %ode15s for stiff problems
    [t,x,te,xe,ie] = ode45(@SystemSimulation, tspan, x0, options);
    nt = length(t);
    time = [time;t(2:end)];
    xnew = [xnew;x(2:end,:)]; %store results vector
    teout = [teout; te];    % Events at tstart are never reported.
    xeout = [xeout; xe];
    ieout = [ieout; ie];
    e=.6 %Coefficient of restitution
    for i = 1:26
        oldx0(i,1) = x(end,i); %conditions right before or at impact
    end
    [x0] = Collision(oldx0,e); %new initial conditions after impact
   % A good guess of a valid first time step is the length of 
    % the last valid time step, so use it for faster computation.
%     options = odeset(options,'InitialStep',t(end)-t(1:nt-1));
    t0 = t(end);
    options = odeset(options,'InitialStep',.0001,...
                           'MaxStep',(t(nt)-t(1)));
    if t0 >= tf-h
        break;
    end
    
end


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
j=1;
s0 = sqrt((SRD(j,4)-SRD(j,1))^2 + (SRD(j,5)-SRD(j,2))^2 + (SRD(j,6)-SRD(j,3))^2)+drop; 
for j = 1:i
    [phi1(j,1),theta1(j,1),psi1(j,1)] = DCM2Euler(Boep(j,1),B1ep(j,1),B2ep(j,1),B3ep(j,1));
    [phi2(j,1),theta2(j,1),psi2(j,1)] = DCM2Euler(Boec(j,1),B1ec(j,1),B2ec(j,1),B3ec(j,1));
    [ForceX(j,:), ForceY(j,:), ForceZ(j,:), ForceTotal(j,:),Torques(j,:), SumForce(j,:),...
    Normal(j,:), CargoPointsX(j,:),CargoPointsY(j,:),CargoPointsZ(j,:), stroke(j,:), Detection(j,:), Def(j,:)] = ForceCalcs(j);

end

save 500testing.mat
            
%MOVIES!
% SRDMovie(0,0) %side
% SRDMovie(90,0) %front
% SRDMovie(15,15) %iso
% SRDMovie(0,90) %top


