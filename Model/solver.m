%Run SRDmovie.m to create a movie of the results

%Global variables passed to the various sub-modules

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
       maxstroke RipForce1 totalMB1 totalMB2 totalMB3 total463L...
       totalSRDweight total_lanyard_weight check type switches horz
 
count = 1;

sign = 1;
noMB1 = 1; %10K
noMB2 = 1;  %for 5 kip rings
noMB3 = 1; %15K
noMB4 = 1; %20K
no463L = 1; %7.5K
noMB5 = 1; %25K
holden = 1;
increment = 1;

%For those small 200-lb red SRDs we drop tested
%V-138 Poly
%Mil-spec 5625 Nylon


%Failure of SRD = failure of mil-5625 nylon webbing (4000 lb)
Fmax1 = noMB1 * 52 * 17792.8864; 
Fmax2 = noMB2 * 26 * 17792.8864; 

% Kkevlar = 44482.216/(lanyard*conv*0.05/kfactor);

offsetx = 0;
offsety = 0;
offsetz = 1*conv;
Oui = 1;
Non = 0;

SRD = [ 
 -L/2  W/2  0  -b/2  a/2  h K1*noMB1 RipForce1*noMB1 Oui;... 
 -L/2 -W/2  0  -b/2 -a/2  h K1*noMB1 RipForce1*noMB1 Oui;...
  L/2  W/2  0   b/2  a/2  h K1*noMB1 RipForce1*noMB1 Oui;...
  L/2 -W/2  0   b/2 -a/2  h K1*noMB1 RipForce1*noMB1 Oui];

%%%%%%%%%%%%%%% Initial Conditions for Platform %%%%%%%%%%%%%%%
x01 = 0; %X location of Platform/Aircraft [m]
y01 = 0; %Y location of Platform/Aircraft [m]
z01 = 0; %Z location of Platform/Aircraft [m]
% v0x1 = 0; %X velocity of Platform/Aircraft [m/s]
% v0y1 = 6.4; %Y velocity of Platform/Aircraft [m/s]
% v0z1 = 0; %Z velocity of Platform/Aircraft [m/s]
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
% v0z2 = 0; %Z velocity of Cargo [m/s]
phi02 = 0; %Rotation of Cargo about X axis [rad]
theta02 = 0; %Rotation of Cargo about Y axis [rad]
psi02 = 0; %Rotation of Cargo about Z axis [rad]
p02 = 0; %Angular velocity about X axis of Cargo [rad/s]
q02 = 0; %Angular velocity about Y axis of Cargo [rad/s]
r02 = 0; %Angular velocity about Z axis of Cargo [rad/s]
       
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
        smax(j,1) = maxstroke(j)*.3048 + (SRD(j,7)/SRD(j,8));
        s0(j,1) = sqrt((SRD(j,4)-SRD(j,1)+x02)^2 + (SRD(j,5)-SRD(j,2)+y02)^2 + (SRD(j,6)-SRD(j,3)+z02)^2);
    end
end %calc smax

%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%
%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%
%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%

t0 = 0; %initial time
%initial conditions vector
x0 = [x_01;y_01;z_01;B0ep;p_01;q_01;r_01;x_02;y_02;z_02;B0ec;p_02;q_02;r_02]; 
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
    options = odeset(options,'InitialStep',.0001,...
                           'MaxStep',(t(nt)-t(1)));
    if t0 >= tf
        break;
    end
end %SOLVER

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
numbers = [noMB1 noMB2 no463L noMB4]
min(u2-u1)
max(x2-x1)
lanyardweight = zeros(n,1);

if type == 1
    for k = 1:n
        if SRD(k,end) == 1
            smax(k,1) = maxstroke(k)*.3048 + (SRD(k,7)/SRD(k,8));
            s0(k,1) = sqrt((SRD(k,4)-SRD(k,1)+x02)^2 + (SRD(k,5)-SRD(k,2)+y02)^2 +...
            (SRD(k,6)-SRD(k,3)+z02)^2);
            lanyardweight(k,1) = chain; %chains are always same lenght and same mass, you can't have shorter/longer ones
            if SRD(k,7) == RipForce1*noMB3*1.5
                lanyardweight(k,1) = chain2;
            elseif SRD(k,7) == RipForce1*2*noMB4
                lanyardweight(k,1) = chain2;
            elseif SRD(k,7) == RipForce1*2.5*noMB5
                lanyardweight(k,1) = chain2;
            end
        end
    end%reset smax
elseif type == 2 
    for k = 1:n
            if SRD(k,end) == 1
                smax(k,1) = maxstroke(k)*.3048 + (SRD(k,7)/SRD(k,8));
                s0(k,1) = sqrt((SRD(k,4)-SRD(k,1)+x02)^2 + (SRD(k,5)-SRD(k,2)+y02)^2 +...
                (SRD(k,6)-SRD(k,3)+z02)^2);
                lanyardweight(k,1) = 2*nylon; %nylon is originally for 5 kip so scale up
                if SRD(k,7) == RipForce1*noMB3*1.5
                    lanyardweight(k,1) = nylon*3;
                elseif SRD(k,7) == RipForce1*2*noMB4
                    lanyardweight(k,1) = nylon*4;
                elseif SRD(k,7) == RipForce1*2.5*noMB5
                    lanyardweight(k,1) = nylon*5;
                end
            end
    end%reset smax
end

if check == 1
    for j = 1:i
    %     [phi1(j,1),theta1(j,1),psi1(j,1)] = DCM2Euler(Boep(j,1),B1ep(j,1),B2ep(j,1),B3ep(j,1));
    %     [phi2(j,1),theta2(j,1),psi2(j,1)] = DCM2Euler(Boec(j,1),B1ec(j,1),B2ec(j,1),B3ec(j,1));

    %     [ForceX(j,:), ForceY(j,:), ForceZ(j,:), ForceTotal(j,:),Torques(j,:), SumForce(j,:),...
    %     Normal(j,:), CargoPointsX(j,:),CargoPointsY(j,:),CargoPointsZ(j,:), stroke(j,:), newstroke(j,:), Detection(j,:), Def(j,:)] = ForceCalcs(j);
        [ForceTotal(j,:),newstroke(j,:)] = InterVar(j);
    end%recalculate mid-variables
end

countMB1 = 0; %10k
countMB2 = 0; %5k
countMB3 = 0; %15k
countMB4 = 0; %20k
count463L = 0; %7.5k
countMB5 = 0; %25k
weightMB1 = 0;
weightMB2 = 0;
weightMB3 = 0;
weightMB4 = 0;
weight463L = 0;
weightMB5 = 0;

%VARIABLE 'maxstroke' is in FEET not METERS

for j = 1:n
    if SRD(j,end) == 1
        if SRD(j,7) == RipForce1*noMB1
            countMB1 = countMB1 + 1;
            weightMB1 = weightMB1 + (maxstroke(j)*1);
        elseif SRD(j,7) == RipForce1*0.5*noMB2
            countMB2 = countMB2 + 1;
            weightMB2 = weightMB2 + (maxstroke(j)*0.5);
        elseif SRD(j,7) == RipForce1*0.75*no463L
            count463L = count463L + 1;
            weight463L = weight463L + (maxstroke(j)*0.75);
        elseif SRD(j,7) == RipForce1*1.5*noMB3
            countMB3 = countMB3 + 1;
            weightMB3 = weightMB3 + (maxstroke(j)*1.5);
        elseif SRD(j,7) == RipForce1*2*noMB4
            countMB4 = countMB4 + 1;
            weightMB4 = weightMB4 + (maxstroke(j)*2);
        elseif SRD(j,7) == RipForce1*2.5*noMB5
            countMB5 = countMB5 + 1;
            weightMB5 = weightMB5 + (maxstroke(j)*2.5);
        end
    end
end
total_lanyard_weight = sum(lanyardweight);
% totalSRDweight = (weightMB1 + weightMB2 + weightMB3 + weightMB4 + weight463L)*39.5/1000;
totalSRDweight = (weightMB1 + weightMB2 + weightMB3 + weightMB4 + weight463L + weightMB5)*0.73;
%A tear webbing with exactly 10 kip activation force and 1ft stroke weighs
%0.73 kg

totalweight = (total_lanyard_weight + totalSRDweight); %we're just interested in the half that applies for aft so we divide by 2

totalMB1 = countMB1;
totalMB2 = countMB2;
totalMB3 = countMB3;
total463L = count463L;
totalMB4 = countMB4;
totalMB5 = countMB5;



