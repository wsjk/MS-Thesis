function [value,isterminal,direction] = events(t,x)

global m1 m2 g a b h h2 W L g...
       smax RipForce K1 K2 K3 C...
       x1 y1 z1 Boep B1ep B2ep B3ep p1 q1 r1...
       x2 y2 z2 Boec B1ec B2ec B3ec p2 q2 r2...   
       u1 v1 w1 u2 v2 w2...
       SRD Cargo Platform... 
       Jz1 Jx1 Jy1 Jz2 Jx2 Jy2...
       count mu broken Fmax whom Contact fail

hit = 0;
% Locate the time when cargo hits platform
DCMpe = Quaternions2DCM(x(7),x(8),x(9),x(10),1); %Platform -> Earth
DCMep = Quaternions2DCM(x(7),x(8),x(9),x(10),2); %Earth to Platform CS
DCMce = Quaternions2DCM(x(20),x(21),x(22),x(23),1); %Cargo -> Earth
DCMec = Quaternions2DCM(x(20),x(21),x(22),x(23),2); %Earth to Cargo CS 
Contact = zeros(1,8);
for j = 1:8 %for the corners of cargo/platform: cargo/platform is a solid, rigid box
    %Px_rp = Rotation of Platform corners due to rotation
    Px_rp = DCMpe * [Platform(j,1); Platform(j,2); 0]; 
    %Px_e = Location of Platform corners in Earth fixed cs
    Px_e = [x(1); x(3); x(5)] + Px_rp; 
    %Cx_rc = Rotation of Cargo corners in Earth fixed cs
    Cx_rc = DCMce * [Cargo(j,1); Cargo(j,2); Cargo(j,3)];
    %Cx_e = Location of Cargo corners in Earth fixed cs
    Cx_e = [x(14); x(16); x(18)] + Cx_rc; 
    %Displacement of Cargo corners from Platform in Platform cs (for collision)
    Cloc = DCMep * (Cx_e - Px_e);
    Cx(j,1) = Cloc(1,1);
    Cy(j,1) = Cloc(2,1);
    Cz(j,1) = Cloc(3,1);
    if Cz(j,1) <= 1e-3 %Contact detected
        Contact(1,j) = 1;
    else
        Contact(1,j) = 0;
    end
end

if sum(Contact) > 0
    hit = 1;
else
    hit = 0;
end
for j = 1:8
    hit = 1-Contact(1,j);
    value = hit;    % Detect height = 0
    isterminal = 1;   % Stop the integration
    direction = -1;   % Negative direction only
    if hit == 0
        t(end);
        Contactwho=Contact;
        break;
    end
end
Contactlast = Contact;

return;