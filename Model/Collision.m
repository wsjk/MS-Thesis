    function [newx0] = Collision(x0,e,Contactnow)

global m1 m2 g b h h2 W L g...
       smax RipForce K1 K2 K3 C...
       x1 y1 z1 Boep B1ep B2ep B3ep p1 q1 r1...
       x2 y2 z2 Boec B1ec B2ec B3ec p2 q2 r2...   
       u1 v1 w1 u2 v2 w2...
       GRID Cargo Platform... 
       Jz1 Jx1 Jy1 Jz2 Jx2 Jy2...
       count mu broken Fmax whom Ip Ic...
       Contact TimeOfContact

%Current orientation of cargo and platform
DCMpe = Quaternions2DCM(x0(7,1),x0(8,1),x0(9,1),x0(10,1),1); %Platform -> Earth
DCMep = Quaternions2DCM(x0(7,1),x0(8,1),x0(9,1),x0(10,1),2); %Earth to Platform CS
DCMce = Quaternions2DCM(x0(20,1),x0(21,1),x0(22,1),x0(23,1),1); %Cargo -> Earth
DCMec = Quaternions2DCM(x0(20,1),x0(21,1),x0(22,1),x0(23,1),2); %Earth to Cargo CS 
Contact = zeros(1,8);
for j = 1:8 %for the corners of cargo/platform: cargo/platform is a solid, rigid box
    %Px_rp = Rotation of Platform corners due to rotation
    Px_rp = DCMpe * [Platform(j,1); Platform(j,2); 0]; 
    %Px_e = Location of Platform corners in Earth fixed cs
    Px_e = [x0(1,1); x0(3,1); x0(5,1)] + Px_rp; 
    %Cx_rc = Rotation of Cargo corners in Earth fixed cs
    Cx_rc = DCMce * [Cargo(j,1); Cargo(j,2); Cargo(j,3)];
    %Cx_e = Location of Cargo corners in Earth fixed cs
    Cx_e = [x0(14,1); x0(16,1); x0(18,1)] + Cx_rc; 
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

Contactnow = Contact;
j = 1;
side = 'none';
%Determine how many and where the points of contact are
if sum(Contactnow) > 1
%     If two adjacent corners of the cargo hit the platform, we change
%     impact point to be midpoint of the two corners.
    if Contactnow(1,2) == 1 && Contactnow(1,4) == 1
        side = 'aft'; %negative x direction
    elseif Contactnow(1,6) == 1 && Contactnow(1,8) == 1
        side = 'forward'; %positive x direction
    elseif Contactnow(1,2) == 1 && Contactnow(1,8) == 1
        side = 'starboard'; %right (negative y direction)
    elseif Contactnow(1,4) == 1 && Contactnow(1,6) == 1
        side = 'port'; %left(positive y direction)
    end
    if sum(Contactnow) == 4
        side = 'all';
    end
elseif sum(Contactnow) == 1
    side = 'single';
    for i = 1:8
        if Contactnow(1,i) == 1
            a = i;
            Rce = DCMce*[Cargo(a,1); Cargo(a,2); Cargo(a,3)]; 
            break;
        end
    end
end

%Determine vector from cargo cg to point of contact, which is a cargo
%corner unless more than one corner is in contact.
% Rce = location of cargo corners relative to cargo cg in cargo cs
if sum(Contactnow) > 0 
    switch lower(side)
        case 'aft'
            a = 2;
            Rce = DCMce * [Cargo(a,1); 0; Cargo(a,3)];
        case 'forward'
            a = 6;
            Rce = DCMce * [Cargo(a,1); 0; Cargo(a,3)];
        case 'starboard'
            a = 8;
            Rce = DCMce*[0; Cargo(a,2); Cargo(a,3)];       
        case 'port'
            a = 6;
            Rce = DCMce*[0; Cargo(a,2); Cargo(a,3)];
        case 'all'
            a = 1;
            Rce = DCMce*[0;0;Cargo(a,3)]; %all 4 corners in contact
        otherwise
            Rce = DCMce*[Cargo(a,1); Cargo(a,2); Cargo(a,3)]; 
    end

    %p,q and r are already in earth fixed cs
    wpe1 = (DCMpe*[x0(11); x0(12); x0(13)])'; %angular velocity of platform in earth fixed cs. 
    % wpe1 = [0; 0; 0]; %angular velocity of platform in earth fixed cs. 

    %Rpe (position of platform corners) isn't important since the
    %corner of the cargo will never hit a corner of the platform
    %wce1 = angular velocity of cargo in earth fixed cs
    wce1 = (DCMce*[x0(24); x0(25); x0(26)])';

    %Pp_e = Vector from Platform CG to point of contact on Platform in 
    %earth fixed inertial frame 
    %Pp_e = Rce - (x1;y1;z1)
    Ppxe(a) = Rce(1,1) - x0(1); 
    Ppye(a) = Rce(2,1) - x0(3); 
    Ppze(a) = Rce(3,1) - x0(5);

    %Pc = location of point of contact wrt to cargo CG, 
    %because impact is always going to be on corner of cargo
    Pc = Rce';
    %Pp = location of point of contact on platform in earth fixed CS
    Pp = [Ppxe(a) Ppye(a) Ppze(a)];
    %n1 = unit vector normal to the platform surface in inertial frame
    %collision means a cargo corner is hitting the surface of the
    %platform
    n1 = (DCMpe*[0;0;1])'; %vector normal to surface of impact (i.e. platform)  
    %Vpe1 = Velocity of contact point on platform 
    Vpe1 = [x0(2) x0(4) x0(6)] + cross(wpe1,Pp);
    %Vce1 = Velocity of contact point on cargo (i.e. cargo corner) 
    Vce1 = [x0(15) x0(17) x0(19)] + cross(wce1,Pc);
    vcp = Vce1 - Vpe1; %relative velocity, cargo is colliding into platform
    Vrel = dot(vcp,n1) %Component of relative velocity in the direction normal to platform surface

    Ice = DCMce*Ic*DCMce'; %Cargo inertia tensor in earth fixed frame
    Ipe = DCMpe*Ip*DCMpe'; %Platform inertia tensor in earth fixed frame
    % if x0(5,1) <= 0
        %J = impulse parameter for when platform is not affected by collision
        %(i.e. m1 = infinite mass)
        J = -(1+e)*Vrel / (1/m2 + dot(n1, cross( (inv(Ice) * (cross(Pc,n1))')', Pc)))   ;
    % else
    %     J = -(1+e)*Vrel /...
    %          (1/m1 + dot(n1, cross( (inv(Ipe) * (cross(Pp,n1))')', Pp))...
    %         + 1/m2 + dot(n1, cross( (inv(Ice) * (cross(Pc,n1))')', Pc)))
    % end

    % Vpe2 = Vpe1 - J*n1/m1 %velocity of platform CG after impact
    % wpe2 = wpe1 - (inv(Ipe) * cross(J,Pp)') %angular velocity of platform
    % after impact
    Vpe2 = [0 0 0];
    wpe2 = [0 0 0];
    Vce2 = Vce1 + J*n1/m2; %velocity of cargo CG after impact
    wce2 = wce1 + (inv(Ice) * cross(Pc,J*n1)')'; %angular velocity of cargo after impact

    %Vpe2 = (u1 v1 w1)
    %Vce2 = (u2 v2 w2)
    %wp2 = (p1 r1 q1)
    %wc2 = (p2 r2 q2)
    newx0 = x0;
    newx0(2,1) = Vpe2(1,1);
    newx0(4,1) = Vpe2(1,2);
    newx0(6,1) = Vpe2(1,3);
    newx0(15,1) = Vce2(1,1);
    newx0(17,1) = Vce2(1,2);
    newx0(19,1) = Vce2(1,3);
    newx0(11,1) = wpe2(1,1);
    newx0(12,1) = wpe2(1,2);
    newx0(13,1) = wpe2(1,3);
    newx0(24,1) = wce2(1,1);
    newx0(25,1) = wce2(1,2);
    newx0(26,1) = wce2(1,3);
elseif sum(Contactnow) == 0
    newx0 = x0;
end
    

return;