

%Recalculate forces because Matlab ODE solvers won't return intermediate
% %variables.
function [ForceX, ForceY, ForceZ, ForceTotal,Torques, SumForce, Normal, CargoPointsX,CargoPointsY,CargoPointsZ, stroke, Detection, Def] = ForceCalcs(t)

global m1 m2 g h...
       Fmax smax smax2 RipForce K1 K2 K3 C...
       Jz1 Jx1 Jy1 Jz2 Jx2 Jy2...
       SRD Cargo Platform...
       x1 y1 z1 Boep B1ep B2ep B3ep p1 q1 r1...
       x2 y2 z2 Boec B1ec B2ec B3ec p2 q2 r2...
       u1 v1 w1 u2 v2 w2...
       count mu broken s0

[n,m] = size(SRD); %n = total number of SRDs 
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

%Calculating displacment and location of SRD attachment point on the cargo
% [x2 y2 z2]%Transform translations from earth fixed to cargo 
%dx2, dy2, dz2 = distance in x,y,z directions of SRD cargo attachment point 
%relative to SRD floor attachment point in platform cs
x_rc1 = zeros(3,1);
x_rc2 = zeros(3,1);
x_e1 = zeros(3,1);
x_e2 = zeros(3,1);
dx2 = zeros(n,1); %X-distance between SRD attachment point on cargo and on platform
dy2 = zeros(n,1); %Y-distance between SRD attachment point on cargo and on platform
dz2 = zeros(n,1); %Z-distance between SRD attachment point on cargo and on platform
dxp = zeros(n,3); %Unit vector between SRD attachment point on cargo and on platform
dx = zeros(n,3);
for j = 1:n %for the number of SRDs  
    %x_rc1 = Rotation of SRD floor attachment vector due to rotation relative to earth
    x_rc1 = DCMpe * [SRD(j,1);SRD(j,2);SRD(j,3)];
    %x_e1 = Location of SRD floor attachment point in Earth fixed cs
    x_e1 = [x1(t); y1(t); z1(t)] + x_rc1;
    %x_rc2 = Rotation of SRD cargo attachment vector from cargo to Earth fixed cs
    x_rc2 = DCMce * [SRD(j,4);SRD(j,5);SRD(j,6)];
    %x_e2 = %Location of SRD cargo attachment point in Earth fixed cs
    x_e2 = [x2(t); y2(t); z2(t)] + x_rc2;
    %Displacement of SRD attachment point on Cargo relative to attachment point on Platform
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
end

%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|
%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|
%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|/%%\|

S = zeros(n,1); %Stroke of SRDs, positive stroke means SRD is in tension
newS = zeros(n,1);
FX = zeros(n,1); %X component of SRD forces on Cargo in aircraft/platform cs
FY = zeros(n,1); %Y component of SRD forces on Cargo in aircraft/platform cs
FZ = zeros(n,1); %Z component of SRD forces on Cargo in aircraft/platform cs
Mxc = zeros(n,1); %Moments about cargo CG along x axis in cargo cs
Myc = zeros(n,1); %Moments about cargo CG along y axis in cargo cs
Mzc = zeros(n,1); %Moments about cargo CG along z axis in cargo cs
MNX = zeros(8,1); %Moments about cargo CG along x axis due to normal force in cargo cs
MNY = zeros(8,1); %Moments about cargo CG along x axis due to normal force in cargo cs
MNZ = zeros(8,1); %Moments about cargo CG along x axis due to normal force in cargo cs
Ftot = zeros(n,1); %Resultant of X and Y components of SRD forces 
%SRD force prior to activation, acts as a linear spring of stiffness K1
newS = zeros(n,1); %keep track of SRD stroke that can't retract 
K=0;
for j = 1:n
    %s0 = %Original length of SRD
%     s0 = sqrt((SRD(j,4)-SRD(j,1))^2 + (SRD(j,5)-SRD(j,2))^2 + (SRD(j,6)-SRD(j,3))^2);  
    S(j,1) = sqrt(dx2(j,1)^2 + dy2(j,1)^2 + dz2(j,1)^2) - s0;
    zero = 1;
    state = 0;
    if broken(j,1) == 5 %if it's already broken then it can't do anything ever again
        zero = 0;
%         state = 5;
    end
    if S(j,1) <= 0 %SRD can not take compressive loads, negative stroke = no SRD force
        zero = 0;
    end
    if S(j,1) > 0
        K = K1;
        if S > smax
            K = K2;
        end
        if zero == 1
            [FX(j,1), FY(j,1), FZ(j,1),state] = RestraintForces(K,dx(j,:),S(j,1));
        end
    end
    broken(j,1) = state;
    Ftot(j,1) = sqrt(FX(j,1)^2 + FY(j,1)^2 + FZ(j,1)^2);
end

%Moment on Cargo due to SRDs
for j = 1:n
    F_p = [FX(j,1);FY(j,1);FZ(j,1)]; %Force vector in aircraft/platform cs
    F_c = DCMec * DCMpe * F_p; %Force vector in cargo cs
    r_c = [SRD(j,4) SRD(j,5) SRD(j,6)];
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
NC = zeros(8,3);
NP = zeros(8,3);
N_c = zeros(3,1);
hit = zeros(1,8);
sink = zeros(1,8);
for j = 1:8
  if location(j,2) == 1
      %velpe = Velocity of the corners of the platform in earth fixed cs
      velpe = [u1(t) v1(t) w1(t)] + cross([p1(t) q1(t) r1(t)],[Platform(j,1) Platform(j,2) Platform(j,3)]);
      %velce = Velocity of the corners of the cargo in earth fixed cs
      velce = [u2(t) v2(t) w2(t)] + cross([p2(t) q2(t) r2(t)],[Cargo(j,1) Cargo(j,2) Cargo(j,3)]);
      Impact = DCMep * (velce' - velpe'); %Impact velocity of cargo relative to platform in earth fixed cs
      if abs(Impact(3,1)) <= 1e-3
            NP(j,:) = (N_p - [0;0;C*Impact(3,1)] - K3*[0;0;Cz(j,1)])';
            resting(1,j) = 1;
      end
      N_c = DCMec*DCMpe*(NP(j,:)'); %Normal force from SRD/weight + Collision
      NC(j,:) = N_c';
      r_N = [Cargo(j,1) Cargo(j,2) Cargo(j,3)];
      %Moments on cargo due to normal force, positive in counterclockwise direction 
      M_N = cross(r_N,N_c');
      %Moments on cargo due to normal force, positive in counterclockwise direction 
      MNX(j,1) = M_N(1,1);
      MNY(j,1) = M_N(1,2);
      MNZ(j,1) = M_N(1,3); 
      hit(1,j) = Impact(3,1);
      sink(1,j) = Cz(j,1);
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
Detection = hit;
Def = sink;

return;