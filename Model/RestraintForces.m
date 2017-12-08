function [FX,FY,FZ] = RestraintForces(K,dx,S,j,t)

global m1 m2 g h...
       Fmax1 Fmax2 smax smax2 RipForce K1 K2 K3 C...
       Jz1 Jx1 Jy1 Jz2 Jx2 Jy2...
       GRID Cargo Platform...
       x1 y1 z1 Boep B1ep B2ep B3ep p1 q1 r1...
       x2 y2 z2 Boec B1ec B2ec B3ec p2 q2 r2...
       u1 v1 w1 u2 v2 w2...
       count mu  
   
    FX = -K * S * dx(1,1); %X component of load limiter force in aircraft/platform cs
    FY = -K * S * dx(1,2); %Y component of load limiter force in aircraft/platform cs
    FZ = -K * S * dx(1,3); %Z component of load limiter force  
    Ftot = sqrt(FX^2 + FY^2 + FZ^2); %resultant load limiter force
    
    %if load limiter force is greater than activation force then we limit the load
    if Ftot >= GRID(j,7) 
        FX = -GRID(j,7) * dx(1,1);
        FY = -GRID(j,7) * dx(1,2); 
        FZ = -GRID(j,7) * dx(1,3); 
    end
    
    %if load limiter is past max stroke, load limiter has failed
    if S >= smax(j,1)
        FX = 0;
        FY = 0;
        FZ = 0;
    end
    
    Ftot3 = sqrt(FX^2 + FY^2 + FZ^2); %resultant load limiter force
return;