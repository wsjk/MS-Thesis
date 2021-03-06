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
       Contact TimeOfContact Ground tstop s0 fail timestep

TimeOfContact = 0;   
Contact = zeros(1,8);
count = 1;

g = 9.8; %gravitational constant [m/s^2]
m1 = 70000; %platform (i.e. aircraft) [kg]
% m2 = 18000; %cargo [kg] (40 kips)
% m2 = 30000*4.4482216/9.81; %cargo [N] (CH-53E)
m2 = 0.45359237 * 65
a = 8; %Width of cargo (y-direction) [m]
b = 8; %Length of cargo (x-direction) [m]
h = 1;  %Height of SRD attachment on cargo (z-direction) [m]
h2 = .25; %Thickness of platform which the cargo is on top of [m]
W = 20; %Width of Platform (Aircraft) [m]
L = 20; %Length of Platform [m]

% maxF = zeros(25,2);
% keep=0;
% sign = 1;
% while(L==20)
% keep = keep+1;
% noSRD = 3*350+50*keep
% 
% %For those small 200-lb red SRDs we drop tested
% noSRD = 600;
% maxstroke = 3;
% Fmax = noSRD * 17792.8864; %Failure of SRD = failure of mil-5625 nylon webbing (4000 lb)
% RipForce = 840*noSRD; %Activation force of SRD [N]
% K1 = 28400*noSRD; %Spring constant of SRD before activation [N/m]
% K2 = 27000*noSRD; %Spring constant of SRD after fully stroked [N/m]

%From ARM Report tests of the black/yellow nylon/polyester 3" wide SRD
%poly_nylon screamer-346 thread_32 rows.xls
% noSRD = 240
% maxstroke = 2; %[m] max stroke of SRD
% Fmax = 11500*4.4482216*noSRD; %[N] From ARM poly_nylon screamer web failure test thread
% weight = (23/300)*noSRD*2 %[lb/ft]*noSRD*maxstroke, weight of webbing, 100 yd spool is 23 lb shipping weight
% RipForce = 1770*4.4482216*noSRD; %Activation force of SRD [N]
% K1 = noSRD * 619.07 * 4.4482216 * 39.37007874; %[N/m] Stiffness of SRD prior to activation 
% K2 = 2311.6*4.4482216*noSRD; %[N/m] from the ARM poly_nylon screamer web failure test 
% K2 = noSRD*K1*0.90;

noSRD = 1;
maxstroke = 1;
Fmax = noSRD * 17792.8864; %Failure of SRD = failure of mil-5625 nylon webbing (4000 lb)
RipForce = 840*noSRD; %Activation force of SRD [N]
K1 = 28400*noSRD; %Spring constant of SRD before activation [N/m]
K2 = 27000*noSRD; %Spring constant of SRD after fully stroked [N/m]

drop = 0.5 + 30 * 0.0254

K3 = 1e9; %Spring Constant for interaction between cargo and platform [N/m]
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
%         -SRDx    SRDy  0   -4    4   h]; %SRD4 (bottom right corner)
     
SRD = [ 0  0  0   0   0   h/2]
     

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
smax = maxstroke*.3048*ones(n,1);
fail = zeros(n,1);
for j = 1:n
    s0(j,1) = sqrt((SRD(j,4)-SRD(j,1))^2 + (SRD(j,5)-SRD(j,2))^2 + (SRD(j,6)-SRD(j,3))^2)+drop;
end

%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%
%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%
%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%

t0 = 0;
timestep = 1/10000;
tf = 1
tstop = 0; %Time when platform hits wall. NOTE: if tstop = 0, then platform does not hit wall
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
    e=.6; %Coefficient of restitution
    for i = 1:26
        oldx0(i,1) = x(end,i); %conditions right before or at impact
    end
    [x0] = Collision(oldx0,e); %new initial conditions after impact
%     x0 = oldx0;
%     K = K2;
    
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
smax = maxstroke*.3048*ones(n,1);
fail = zeros(n,1);
for k = 1:n
    s0(k,1) = sqrt((SRD(k,4)-SRD(k,1))^2 + (SRD(k,5)-SRD(k,2))^2 + (SRD(k,6)-SRD(k,3))^2)+drop;
end
for j = 1:i
    [phi1(j,1),theta1(j,1),psi1(j,1)] = DCM2Euler(Boep(j,1),B1ep(j,1),B2ep(j,1),B3ep(j,1));
    [phi2(j,1),theta2(j,1),psi2(j,1)] = DCM2Euler(Boec(j,1),B1ec(j,1),B2ec(j,1),B3ec(j,1));
    [ForceX(j,:), ForceY(j,:), ForceZ(j,:), ForceTotal(j,:),Torques(j,:), SumForce(j,:),...
    Normal(j,:), CargoPointsX(j,:),CargoPointsY(j,:),CargoPointsZ(j,:), stroke(j,:), newstroke(j,:), Detection(j,:), Def(j,:)] = ForceCalcs(j);
end

min(u2)
% if min(u2) < 1
% %     maxF(sign,1) = noSRD;
% %     maxF(sign,2) = max(max(ForceTotal));
% %     maxF
% %     sign = sign+1
%     break;
% end
% 
% if sign == 25
%     break;
% end
% end
% 
% maxF(1,1) = noSRD;
% maxF(1,2) = max(max(ForceTotal));
% maxF

max(newstroke)


save testing.mat
            
%MOVIES!
% SRDMovie(0,0) %side
% SRDMovie(90,0) %front
% SRDMovie(15,15) %iso
% SRDMovie(0,90) %top




% for t0 = 0:h:tf
%     t2 = t0+h;
%     tspan = [t0:h/2:t2]; %Time interval of integration
%     %ode15s for stiff problems
%     [t,x] = ode15s('SystemSimulation', tspan, x0,[],tstop);
%     Contact
%     if sum(Contact) == 0 %if cargo has not hit the platform
%         for i = 1:26
%             %initial conditions vector for next run is same as the end of
%             %current run
%             x0(i,1) = x(end,i); 
%         end
% %         [row,col] = size(x);  
% %       newcol = newcol + col;
%         time = [time;t(2:end)]; %update time vector
%         xnew = [xnew;x(2:end,:)]; %update results vector
%     end    
%     
%     h2 = h/10;
%     if sum(Contact) > 0 %if contact has occured
%         Contactnow = Contact;
%         while (sum(Contact)>0) %find exact time right before contact
%             tspan2 = [t0:h/10:t0+h-h2]; %decrease end of time interval by small increments
%             [t2,x2] = ode15s('SystemSimulation', tspan2, x0,[],tstop); %solve odes
%             %if cargo is no longer in contact then you've backtracked far
%             %enough to find time right before collision
%             if sum(Contact) == 0 
%                 tc = t0+h-h2; %exact time of contact
%                 t0 = tc; %start next simulation from time right before contact
%                 break;
%             end
%             h2 = h2 + h/10;
%         end
%         time = [time;t2(2:end)]; %update time vector
%         xnew = [xnew;x2(2:end,:)]; %update results vector
%         for i = 1:26
%             x0(i,1) = x2(end,i); %conditions right before impact
%         end
%         e=1; %coefficient of restitution
%         %conditions right after impact to be used as initial conditions for
%         %next run
%         [x0] = Collision(x0,e,Contactnow)
%     end
%     time(end)
% end