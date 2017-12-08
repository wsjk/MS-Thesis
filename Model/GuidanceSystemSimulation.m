
function [xdot] = GuidanceSystemSimulation(t,x,flag,tstop)

global m1 m2 g x02 y02 z02...
       smax smax2 RipForce K1 K2 K3 C...
       Jz1 Jx1 Jy1 Jz2 Jx2 Jy2 h...
       GRID Cargo Platform Fmax1 Fmax2 Ic Ip...
       count mu tstop s0 fail pulse peak crash deploy...
       crashX crashY crashtype offsetx offsety offsetz switches 

[n,m] = size(GRID); %n = total number of restraints 
[p,q] = size(Cargo); 
[r,s] = size(Platform);
cgp = Platform(r,1:3); %coordinates of Platform CG in platform cs

%x0 = [x_01;y_01;z_01;B0pe;p_01;q_01;r_01;
%      x_02;y_02;z_02;B0ce;p_02;q_02;r_02];

%Example: x1 = [x1;x1dot] = [displacement; velocity]

%Platform
x1 = x(1:2); %X Position & Velocity of Platform CG in Earth Fixed CS [m]
y1 = x(3:4); %Y Position & Velocity of Platform CG in Earth Fixed CS [m] 
z1 = x(5:6); %Z Position & Velocity of Platform CG in Earth Fixed CS [m]
Boep = x(7); %Euler Parameters, Platform to Earth fixed
B1ep = x(8); %Euler Parameters, Platform to Earth fixed
B2ep = x(9); %Euler Parameters, Platform to Earth fixed
B3ep = x(10);%Euler Parameters, Platform to Earth fixed
p1 = x(11); %Angular rate & acceleration of Platform along X axis about CG [rad/s]
q1 = x(12); %Angular rate & acceleration of Platform along Y axis about CG [rad/s]
r1 = x(13); %Angular rate & acceleration of Platform along Z axis about CG [rad/s]

%Cargo
x2 = x(14:15); %X Location and Velocity of CG of Cargo in Earth Fixed CS [m]
y2 = x(16:17); %Y Location and Velocity of CG of Cargo in Earth Fixed CS [m]
z2 = x(18:19); %Z Location and Velocity of CG of Cargo in Earth Fixed CS [m]
Boec = x(20); %Euler Parameters, Cargo to Earth fixed
B1ec = x(21); %Euler Parameters, Cargo to Earth fixed
B2ec = x(22); %Euler Parameters, Cargo to Earth fixed
B3ec = x(23); %Euler Parameters, Cargo to Earth fixed
p2 = x(24); %Angular rate & acceleration of Cargo along X axis about CG [rad/s]
q2 = x(25); %Angular rate & acceleration of Cargo along X axis about CG [rad/s]
r2 = x(26); %Angular rate & acceleration of Cargo along X axis about CG [rad/s]

%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|
%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|
%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|

%Quaternions -> Direction Cosine Matrix (DCM)
DCMpe = Quaternions2DCM(Boep(1),B1ep(1),B2ep(1),B3ep(1),1); %Platform -> Earth
DCMep = Quaternions2DCM(Boep(1),B1ep(1),B2ep(1),B3ep(1),2); %Earth to Platform CS
DCMce = Quaternions2DCM(Boec(1),B1ec(1),B2ec(1),B3ec(1),1); %Cargo -> Earth
DCMec = Quaternions2DCM(Boec(1),B1ec(1),B2ec(1),B3ec(1),2); %Earth to Cargo CS

%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|
%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|
%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|

%Calculating displacment and location of restraint attachment point on the cargo
% [x2 y2 z2]%Transform translations from earth fixed to cargo 
%dx2, dy2, dz2 = distance in x,y,z directions of restraint cargo attachment point 
%relative to restraint floor attachment point in platform cs
x_rc1 = zeros(3,1);
x_rc2 = zeros(3,1);
x_e1 = zeros(3,1);
x_e2 = zeros(3,1);
dx2 = zeros(n,1); %X-distance between restraint attachment point on cargo and on platform
dy2 = zeros(n,1); %Y-distance between restraint attachment point on cargo and on platform
dz2 = zeros(n,1); %Z-distance between restraint attachment point on cargo and on platform
dx = zeros(n,3); %Unit vector between restraint attachment point on cargo and on platform
for j = 1:n %for the number of restraints  
    if switches(j) == 1
        %x_rc1 = Rotation of restraint floor attachment vector due to rotation relative to earth
        x_rc1 = DCMpe * [GRID(j,1);GRID(j,2);GRID(j,3)];
        %x_e1 = Location of restraint floor attachment point in Earth fixed cs
        x_e1 = [x1(1); y1(1); z1(1)] + x_rc1;
        %x_rc2 = Rotation of restraint cargo attachment vector from cargo to Earth fixed cs
        x_rc2 = DCMce * [GRID(j,4);GRID(j,5);GRID(j,6)];
        %x_e2 = %Location of restraint cargo attachment point in Earth fixed cs
        x_e2 = [x2(1); y2(1); z2(1)] + x_rc2;
        %Displacement of restraint attachment point on Cargo relative to attachment point on Platform
        %in platform cs
        x_p1=DCMep*x_e1;
        x_p2=DCMep*x_e2;
        dx2(j,1)=x_p2(1,1)-x_p1(1,1);
        dy2(j,1)=x_p2(2,1)-x_p1(2,1);
        dz2(j,1)=x_p2(3,1)-x_p1(3,1);
        dx(j,:) = [dx2(j,1) dy2(j,1) dz2(j,1)] / sqrt(dx2(j,1)^2 + dy2(j,1)^2 + dz2(j,1)^2); 
    end
end

Cx = zeros(8,1); %X position of corners of cargo box
Cy = zeros(8,1); %Y position of corners of cargo box
Cz = zeros(8,1); %Z position of corners of cargo box
location = zeros(8,2); %indicates where the cargo is touching platform
CargoVelocity_p = zeros(8,1); %velocity of cargo corners in Platform cs
PlatformVelocity_p = zeros(8,1); %Velocity of Platform corners in Platform cs
%Calculating displacment and location of the corners of the Platform and Cargo
for j = 1:8 %for the corners of cargo/platform: cargo/platform is a solid, rigid box
    %Px_rp = Rotation of Platform corners due to rotation
    Px_rp = DCMpe * [Platform(j,1); Platform(j,2); 0]; 
    %Px_e = Location of Platform corners in Earth fixed cs
    Px_e = [x1(1); y1(1); z1(1)] + Px_rp; 
    %Cx_rc = Rotation of Cargo corners in Earth fixed cs
    Cx_rc = DCMce * [Cargo(j,1); Cargo(j,2); Cargo(j,3)];
    %Cx_e = Location of Cargo corners in Earth fixed cs
    Cx_e = [x2(1); y2(1); z2(1)] + Cx_rc; 
    %Displacement of Cargo corners from Platform in Platform cs (for collision)
    Cloc = DCMep * (Cx_e - Px_e);
    Cx(j,1) = Cloc(1,1);
    Cy(j,1) = Cloc(2,1);
    Cz(j,1) = Cloc(3,1);
    if Cz(j,1) <= 1e-3 %Contact detected
        location(j,1) = j; %Tells us where the cargo is touching the platform
        location(j,2) = 1; %Puts a one in the column at the corner touching platform
    end
end

%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|
%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|
%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|

S = zeros(n,1); %Stroke of restraints, positive stroke means restraint is in tension
FX = zeros(n,1); %X component of restraint forces on Cargo in aircraft/platform cs
FY = zeros(n,1); %Y component of restraint forces on Cargo in aircraft/platform cs
FZ = zeros(n,1); %Z component of restraint forces on Cargo in aircraft/platform cs
Mxc = zeros(n,1); %Moments about cargo CG along x axis in cargo cs
Myc = zeros(n,1); %Moments about cargo CG along y axis in cargo cs
Mzc = zeros(n,1); %Moments about cargo CG along z axis in cargo cs
MNX = zeros(8,1); %Moments about cargo CG along x axis due to normal force in cargo cs
MNY = zeros(8,1); %Moments about cargo CG along y axis due to normal force in cargo cs
MNZ = zeros(8,1); %Moments about cargo CG along z axis due to normal force in cargo cs
Ftot = zeros(n,1); %Resultant of X and Y components of restraint forces 
%restraint force prior to activation, acts as a linear spring of stiffness K1
newS = zeros(n,1); %keep track of restraint stroke that can't retract 

for j = 1:n
    if switches(j) == 1
        %velpe = Velocity of the corners of the platform in earth fixed cs
        velpe = [x1(2) y1(2) z1(2)] + cross([p1(1) q1(1) r1(1)],[GRID(j,1) GRID(j,2) GRID(j,3)]);
        %velce = Velocity of the corners of the cargo in earth fixed cs
        velce = [x2(2) y2(2) z2(2)] + cross([p2(1) q2(1) r2(1)],[GRID(j,4) GRID(j,5) GRID(j,6)]);
        RelVel = DCMep * (velce' - velpe'); %Impact velocity of cargo relative to platform in earth fixed cs
        
        S(j,1) = sqrt(dx2(j,1)^2 + dy2(j,1)^2 + dz2(j,1)^2) - s0(j,1);
        if S(j,1) > 0
            K = GRID(j,8);
            newS(j,1) = S(j,1);
            if S(j,1) >= smax(j,1) 
                newS(j,1) = sqrt(dx2(j,1)^2 + dy2(j,1)^2 + dz2(j,1)^2) - smax(j,1);
                K = 0;
                FX(j,1) = 0;
                FY(j,1) = 0;
                FZ(j,1) = 0;
            end
            if newS(j,1) >= 0 
                [FX(j,1), FY(j,1), FZ(j,1)] = RestraintForces(K,dx(j,:),newS(j,1),j,t);
            end
            if (RelVel(1,1)) <= 1e-6
                if (RelVel(2,1)) <= 1e-6
                    if (RelVel(3,1)) <= 1e-6
%                         if (q2(1)-q1(1)) <= 0.01
                            FX(j,1) = 0;
                            FY(j,1) = 0;
                            FZ(j,1) = 0;
%                         end
                    end
                end
            end
        end
        Ftot(j,1) = sqrt(FX(j,1)^2 + FY(j,1)^2 + FZ(j,1)^2);
    end
end

%Moments on Cargo due to restraints about cargo CG
for j = 1:n
    F_p = [FX(j,1);FY(j,1);FZ(j,1)]; %Force vector in aircraft/platform cs
    F_c = DCMec * DCMpe * F_p; %Force vector in cargo cs
    r_c = [GRID(j,4) GRID(j,5) GRID(j,6)]-[offsetx offsety offsetz];
    M_c = cross(r_c, F_c');
    %Moment has positive in counterclockwise direction 
    Mxc(j,1) = M_c(1,1);
    Myc(j,1) = M_c(1,2);
    Mzc(j,1) = M_c(1,3);
end

%Gravity
weight_e = [0;0;-m2*g]; %Weight of cargo in Earth Fixed CS
%Weight of cargo in Platform cs
weight_p = DCMep * weight_e; 
N = sum(location(:,2) == 1); %Number of corners/wheels of cargo touching floor
if N==0
    N_p = zeros(3,1); %Normal force on Cargo when it isn't touching the platform
else
   %Normal force on cargo, from floor, in platform cs
    N_p = [0;0;-sum(FZ) - weight_p(3,1)] * (1/N);
%     m2*g*(b/2+offsetx)/(b-offsetx)
end

NC = zeros(8,3); %Keeps track of normal force on all corners in cargo cs
NP = zeros(8,3); %Keeps track of normal force on all corners in platform cs
hit = zeros(8,1);
resting = zeros(1,4);
for j = 1:4
    if location(2*j,2) == 1
        %velpe = Velocity of the corners of the platform in earth fixed cs
        velpe = [x1(2) y1(2) z1(2)] + cross([p1(1) q1(1) r1(1)],[Platform(2*j,1) Platform(2*j,2) Platform(2*j,3)]);
        %velce = Velocity of the corners of the cargo in earth fixed cs
        velce = [x2(2) y2(2) z2(2)] + cross([p2(1) q2(1) r2(1)],[Cargo(2*j,1) Cargo(2*j,2) Cargo(2*j,3)]);
        Impact = DCMep * (velce' - velpe'); %Impact velocity of cargo relative to platform in earth fixed cs
        if Impact(3,1) <= 0
            if Cz(2*j,1) <= 1e-5
                NP(j,:) = ((N_p) - [0;0;C*(Impact(3,1))] - K3*[0;0;(Cz(j,1))])';
                resting(1,j) = 1;
            end
        end
%         NP(j,:) = (N_p)';
        N_c = DCMec*DCMpe*(NP(j,:)'); %Normal force from restraint/weight + Collision
        NC(j,:) = N_c'; 
        r_N = [Cargo(2*j,1) Cargo(2*j,2) Cargo(2*j,3)]-[offsetx offsety offsetz];
        M_N = cross(r_N,N_c');
        %Moments on cargo due to normal force, positive in counterclockwise direction 
        MNX(j,1) = M_N(1,1);
        MNY(j,1) = M_N(1,2);
        MNZ(j,1) = M_N(1,3);  
    end
end

Torque_c = [sum(Mxc)+sum(MNX); sum(Myc)+sum(MNY); sum(Mzc)+sum(MNZ)]; %sum of torque on cargo in cargo cs
Torque_e = DCMce * Torque_c; %sum of torque on cargo in earth fixed cs
Forces_p = [sum(FX)+weight_p(1,1); sum(FY)+weight_p(2,1); sum(FZ)+weight_p(3,1)+ sum(NP(:,3))];
Forces_e = DCMpe * Forces_p; %sum of forces on cargo in earth fixed cs
TorqueX = Torque_c(1,1); %sum of torque on cargo in earth fixed cs
TorqueY = Torque_c(2,1);
TorqueZ = Torque_c(3,1);
SumForceX = Forces_e(1,1);%sum of forces on cargo in earth fixed cs
SumForceY = Forces_e(2,1);
SumForceZ = Forces_e(3,1);

x1dot = zeros(2,1); %integrate x1dot(1) = x1, integrate x1dot(2) = v1 = x1(2) 
y1dot = zeros(2,1);
z1dot = zeros(2,1);
p1dot = zeros(1,1);
q1dot = zeros(1,1);
r1dot = zeros(1,1);
Bodot1 = zeros(1,1);
B1dot1 = zeros(1,1);
B2dot1 = zeros(1,1);
B3dot1 = zeros(1,1);
x2dot = zeros(2,1);
y2dot = zeros(2,1);
z2dot = zeros(2,1);
p2dot = zeros(1,1);
q2dot = zeros(1,1);
r2dot = zeros(1,1);
Bodot2 = zeros(1,1);
B1dot2 = zeros(1,1);
B2dot2 = zeros(1,1);
B3dot2 = zeros(1,1);

x1dot(1) = x1(2);
x1dot(2) = -(1/m1) * SumForceX;
y1dot(1) = y1(2);
y1dot(2) = -(1/m1) * SumForceY;
z1dot(1) = z1(2);
% z1dot(2) = -(1/m1) * SumForceZ - g;
z1dot(2) = 0; %if problem starts already on the ground
p1dot(1) = -(1/Jx1) * TorqueX;
q1dot(1) = -(1/Jy1) * TorqueY;
r1dot(1) = -(1/Jz1) * TorqueZ;

Bodot1(1) = 0.5 * (p1(1)*-B1ep(1) + q1(1)*-B2ep(1) + r1(1)*-B3ep(1)); 
B1dot1(1) = 0.5 * (p1(1)* Boep(1) + q1(1)*-B3ep(1) + r1(1)* B2ep(1));
B2dot1(1) = 0.5 * (p1(1)* B3ep(1) + q1(1)* Boep(1) + r1(1)*-B1ep(1));
B3dot1(1) = 0.5 * (p1(1)*-B2ep(1) + q1(1)* B1ep(1) + r1(1)* Boep(1));


if tstop ~= 0
    z1dot(2) = 0;
    z1dot(1) = 0;
    z1dot(2) = 0;
    Bodot1(1) = 0; 
    B1dot1(1) = 0;
    B2dot1(1) = 0;
    B3dot1(1) = 0;            
    p1dot(1) = 0;
    q1dot(1) = 0;
    r1dot(1) = 0;
    
    if crashtype == 1
        if t >= tstop 
            x1dot(1) = x1(2);
            x1dot(2) = -crashX*(t-tstop)/(pulse/2); %put in negative number for decel
            y1dot(1) = y1(2);
            y1dot(2) = -crashY*(t-tstop)/(pulse/2);
        end
        if t >= peak
            x1dot(1) = x1(2);
            x1dot(2) = crashX*(t-peak)/(pulse/2)-crashX; %put in negative number for decel
            y1dot(1) = y1(2);
            y1dot(2) = crashY*(t-peak)/(pulse/2)-crashY;
        end
        if t >= peak + pulse/2
            x1dot(1) = x1(2);
            x1dot(2) = 0;
            y1dot(1) = y1(2);
            y1dot(2) = 0;
        end
    elseif crashtype == 2
        if t >= tstop
            x1dot(1) = x1(2);
            x1dot(2) = -crashX; %put in negative number for decel
            y1dot(1) = y1(2);
            y1dot(2) = -crashY;
        end
        if t >= peak + pulse/2
            x1dot(1) = x1(2);
            x1dot(2) = 0;
            y1dot(1) = y1(2);
            y1dot(2) = 0;
        end
    elseif crashtype == 3
        if t >= tstop
            x1dot(1) = x1(2);
            x1dot(2) = -1e9; %put in negative number for decel
            y1dot(1) = y1(2);
            y1dot(2) = -1e9;
        end
    end
        
    if x1(2) <= 0
        x1dot(1) = x1(2);
        x1dot(2) = 0; %stop the platform from going in reverse
    end
    if y1(2) <= 0
        y1dot(1) = y1(2);
        y1dot(2) = 0;
    end
end

x2dot(1) = x2(2);
x2dot(2) = (1/m2) * SumForceX;
y2dot(1) = y2(2);
y2dot(2) = (1/m2) * SumForceY;
z2dot(1) = z2(2);
z2dot(2) = (1/m2) * SumForceZ;
z2dot(2) = 0;

p2dot(1) = (1/Jx2) * TorqueX;
q2dot(1) = (1/Jy2) * TorqueY;
r2dot(1) = (1/Jz2) * TorqueZ;
 
%Quaternions, Cargo -> Earth Fixed cs
Bodot2(1) = 0.5 * (p2(1)*-B1ec(1) + q2(1)*-B2ec(1) + r2(1)*-B3ec(1)); 
B1dot2(1) = 0.5 * (p2(1)* Boec(1) + q2(1)*-B3ec(1) + r2(1)* B2ec(1));
B2dot2(1) = 0.5 * (p2(1)* B3ec(1) + q2(1)* Boec(1) + r2(1)*-B1ec(1));
B3dot2(1) = 0.5 * (p2(1)*-B2ec(1) + q2(1)* B1ec(1) + r2(1)* Boec(1));

if sum(resting) >= 4
    z2dot(1) = z1dot(1);
    z2dot(2) = z1dot(2);
    p2dot(1) = p1dot(1);
    q2dot(1) = q1dot(1);
    r2dot(1) = r1dot(1);
%     Bodot2(1) = Bodot1(1);
%     B1dot2(1) = B1dot1(1);
%     B2dot2(1) = B2dot1(1);
%     B3dot2(1) = B3dot1(1);
end
    
xdot = [x1dot;y1dot;z1dot;Bodot1;B1dot1;B2dot1;B3dot1;p1dot;q1dot;r1dot;...
        x2dot;y2dot;z2dot;Bodot2;B1dot2;B2dot2;B3dot2;p2dot;q2dot;r2dot];

count = count + 1;

return;