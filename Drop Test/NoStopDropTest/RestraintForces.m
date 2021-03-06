function [FX,FY,FZ,state] = RestraintForces(K,dx,S)

global m1 m2 g h...
       Fmax smax smax2 RipForce K1 K2 K3 C...
       Jz1 Jx1 Jy1 Jz2 Jx2 Jy2...
       SRD Cargo Platform...
       x1 y1 z1 Boep B1ep B2ep B3ep p1 q1 r1...
       x2 y2 z2 Boec B1ec B2ec B3ec p2 q2 r2...
       u1 v1 w1 u2 v2 w2...
       count mu broken
   
    FX = -K * S * dx(1,1); %X component of SRD force in aircraft/platform cs
    FY = -K * S * dx(1,2); %Y component of SRD force in aircraft/platform cs
    FZ = -K * S * dx(1,3); %Z component of SRD force  
    Ftot = sqrt(FX^2 + FY^2 + FZ^2);
    
    state = 0;
    
    if Ftot >= RipForce
        FX = -RipForce * dx(1,1); 
        FY = -RipForce * dx(1,2); 
        FZ = -RipForce * dx(1,3); 
    end
    
    if S >= smax
        FX = -(K*(S-smax) + RipForce) * dx(1,1); 
        FY = -(K*(S-smax) + RipForce) * dx(1,2);
        FZ = -(K*(S-smax) + RipForce) * dx(1,3);
    end
    
    Ftot2 = sqrt(FX^2 + FY^2 + FZ^2);
    
    if Ftot2 >= Fmax
        state = 5;
        FX = 0;
        FY = 0;
        FZ = 0;
    end
    Ftot3 = sqrt(FX^2 + FY^2 + FZ^2);
return;