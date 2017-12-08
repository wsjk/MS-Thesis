

%Recalculate forces because Matlab ODE solvers won't return intermediate
% %variables.
% function [ForceX, ForceY, ForceZ, ForceTotal,Torques, SumForce, Normal, CargoPointsX,CargoPointsY,CargoPointsZ, stroke, newstroke, Detection, Def] = ForceCalcs(t)

function [ForceTotal, newstroke] = InterVar(t)

global m1 m2 g h...
       Fmax1 Fmax2 smax smax2 RipForce K1 K2 K3 C...
       Jz1 Jx1 Jy1 Jz2 Jx2 Jy2...
       GRID Cargo Platform...
       x1 y1 z1 Boep B1ep B2ep B3ep p1 q1 r1...
       x2 y2 z2 Boec B1ec B2ec B3ec p2 q2 r2...
       u1 v1 w1 u2 v2 w2...
       count mu s0 fail deploy offsetx offsety offsetz switches...

[n,m] = size(GRID); %n = total number of restraints 
[p,q] = size(Cargo); 
cgc = Cargo(p,1:3); %coordinates of cargo CG [x y z] in cargo cs
[r,s] = size(Platform);
cgp = Platform(r,1:3); %coordinates of Platform CG in platform cs

%x0 = [x_01;y_01;z_01;B0pe;p_01;q_01;r_01;
%      x_02;y_02;z_02;B0ce;p_02;q_02;r_02];

%Example: x1 = [x1;x1dot] = [displacement; velocity]

%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|
%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|
%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|

%Quaternions -> Direction Cosine Matrix (DCM)
DCMpe = Quaternions2DCM(Boep(t),B1ep(t),B2ep(t),B3ep(t),1); %Platform -> Earth
DCMep = Quaternions2DCM(Boep(t),B1ep(t),B2ep(t),B3ep(t),2); %Earth to Platform CS
DCMce = Quaternions2DCM(Boec(t),B1ec(t),B2ec(t),B3ec(t),1); %Cargo -> Earth
DCMec = Quaternions2DCM(Boec(t),B1ec(t),B2ec(t),B3ec(t),2); %Earth to Cargo CS

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
dxp = zeros(n,3); %Unit vector between restraint attachment point on cargo and on platform
dx = zeros(n,3);
for j = 1:n %for the number of restraints  
    if switches(j) == 1
        %x_rc1 = Rotation of restraint floor attachment vector due to rotation relative to earth
        x_rc1 = DCMpe * [GRID(j,1);GRID(j,2);GRID(j,3)];
        %x_e1 = Location of restraint floor attachment point in Earth fixed cs
        x_e1 = [x1(t); y1(t); z1(t)] + x_rc1;
        %x_rc2 = Rotation of restraint cargo attachment vector from cargo to Earth fixed cs
        x_rc2 = DCMce * [GRID(j,4);GRID(j,5);GRID(j,6)];
        %x_e2 = %Location of restraint cargo attachment point in Earth fixed cs
        x_e2 = [x2(t); y2(t); z2(t)] + x_rc2;
        %Displacement of restraint attachment point on Cargo relative to attachment point on Platform
        %in platform cs
        x_p1=DCMep*x_e1;
        x_p2=DCMep*x_e2;
        dx2(j,1)=x_p2(1,1)-x_p1(1,1);
        dy2(j,1)=x_p2(2,1)-x_p1(2,1);
        dz2(j,1)=x_p2(3,1)-x_p1(3,1);
        dxp(j,:) = (DCMep*x_e2 - DCMep*x_e1)';
        %Displacement unit vector
        dx(j,:) = [dx2(j,1) dy2(j,1) dz2(j,1)] / sqrt(dx2(j,1)^2 + dy2(j,1)^2 + dz2(j,1)^2); 
    end
end

Cx = zeros(8,1); %X position of corners of cargo box
Cy = zeros(8,1); %Y position of corners of cargo box
Cz = zeros(8,1); %Z position of corners of cargo box
CargoPoints = zeros(1,8);
location = zeros(8,2);
CargoVelocity_p = zeros(8,1);
PlatformVelocity_p = zeros(8,1);
%Calculating displacment and location of the corners of the Platform and Cargo
for j = 1:8 %for the corners of cargo/platform: cargo/platform is a solid, rigid box
    %Px_rp = Rotation of Platform corners due to rotation
    Px_rp = DCMpe * [Platform(j,1); Platform(j,2); 0]; 
    %Px_e = Location of Platform corners in Earth fixed cs
    Px_e = [x1(t); y1(t); z1(t)] + Px_rp; 
    %Cx_rc = Rotation of Cargo corners in Earth fixed cs
    Cx_rc = DCMce * [Cargo(j,1); Cargo(j,2); Cargo(j,3)];
    %Cx_e = Location of Cargo corners in Earth fixed cs
    Cx_e = [x2(t); y2(t); z2(t)] + Cx_rc; 
    %Displacement of Cargo corners from Platform in Platform cs (for collision)
    Cloc = DCMep * (Cx_e - Px_e);
    Cx(j,1) = Cloc(1,1);
    Cy(j,1) = Cloc(2,1);
    Cz(j,1) = Cloc(3,1);
    if Cz(j,1) <= 0
        location(j,1) = j; %Tells us where the cargo is touching the platform
        location(j,2) = 1; %Puts a one in the column at the corner touching platform
    end
    CargoPointsX(1,j) = Cx_e(1,1);
    CargoPointsY(1,j) = Cx_e(2,1);
    CargoPointsZ(1,j) = Cx_e(3,1);
    CargoPointsCz(1,j) = Cz(j,1);
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
MNY = zeros(8,1); %Moments about cargo CG along x axis due to normal force in cargo cs
MNZ = zeros(8,1); %Moments about cargo CG along x axis due to normal force in cargo cs
Ftot = zeros(n,1); %Resultant of X and Y components of restraint forces 
%restraint force prior to activation, acts as a linear spring of stiffness K1
newS = zeros(n,1); %keep track of restraint stroke that can't retract 
K=0;
for j = 1:n
    if switches(j) == 1
        %velpe = Velocity of the corners of the platform in earth fixed cs
        velpe = [u1(t) v1(t) w1(t)] + cross([p1(t) q1(t) r1(t)],[GRID(j,1) GRID(j,2) GRID(j,3)]);
        %velce = Velocity of the corners of the cargo in earth fixed cs
        velce = [u2(t) v2(t) w2(t)] + cross([p2(t) q2(t) r2(t)],[GRID(j,4) GRID(j,5) GRID(j,6)]);
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
%                         if (q2(t)-q1(t)) <= 0.5
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

%Moment on Cargo due to restraints
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
    N_p = zeros(3,1);% Normal force on Cargo when it isn't touching the platform
else
    %Normal force on cargo, from floor, in platform cs
    N_p = [0;0;-sum(FZ) - weight_p(3,1)] * (1/N);
end
NC = zeros(4,3);
NP = zeros(4,3);
N_c = zeros(3,1);
hit = zeros(1,4);
sink = zeros(1,4);

for j = 1:4
  if location(2*j,2) == 1
      %velpe = Velocity of the corners of the platform in earth fixed cs
      velpe = [u1(t) v1(t) w1(t)] + cross([p1(t) q1(t) r1(t)],[Platform(2*j,1) Platform(2*j,2) Platform(2*j,3)]);
      %velce = Velocity of the corners of the cargo in earth fixed cs
      velce = [x2(t) y2(t) z2(t)] + cross([p2(t) q2(t) r2(t)],[Cargo(2*j,1) Cargo(2*j,2) Cargo(2*j,3)]);
      Impact = DCMep * (velce' - velpe'); %Impact velocity of cargo relative to platform in earth fixed cs
      if Impact(3,1) <= 0
          if Cz(2*j,1) <= 1e-3
              NP(j,:) = ((N_p) - [0;0;C*(Impact(3,1))] - K3*[0;0;(Cz(j,1))])';
              resting(1,j) = 1;
          end
      end
%       NP(j,:) = (N_p - [0;0;C*Impact(3,1)] - K3*[0;0;Cz(j,1)])';
      N_c = DCMec*DCMpe*(NP(j,:)'); %Normal force from restraint/weight + Collision
      NC(j,:) = N_c';
      r_N = [Cargo(2*j,1) Cargo(2*j,2) Cargo(2*j,3)]-[offsetx offsety offsetz];
      %Moments on cargo due to normal force, positive in counterclockwise direction 
      M_N = cross(r_N,N_c');
      %Moments on cargo due to normal force, positive in counterclockwise direction 
      MNX(j,1) = M_N(1,1);
      MNY(j,1) = M_N(1,2);
      MNZ(j,1) = M_N(1,3); 
      hit(1,j) = Impact(3,1);
      sink(1,j) = Cz(2*j,1);
  end
end

Torque_c = [sum(Mxc)+sum(MNX); sum(Myc)+sum(MNY); sum(Mzc)+sum(MNZ)];
Torque_e = DCMce * Torque_c;
Forces_p = [sum(FX)+weight_p(1,1); sum(FY)+weight_p(2,1); sum(FZ)+weight_p(3,1)+ sum(NP(:,3))];
Forces_e = DCMpe * Forces_p;
Normal = N_c';
ForceX = FX';
ForceY = FY';
ForceZ = FZ';
ForceTotal = Ftot';
Torques = Torque_e';
SumForce = Forces_e';
stroke = S';
newstroke = newS';
Detection = hit;
Def = sink;

return;