function [phi,theta,psi] = DCM2Euler(Bo,B1,B2,B3)
%Direction Cosine Matrix -> Euler Angles

%Earth to body (i.e. cargo,platform) CS
DCM = [Bo^2+B1^2-B2^2-B3^2, 2*(B1*B2+Bo*B3), 2*B1*B3-Bo*B2;...
       2*(B1*B2-Bo*B3), Bo^2-B1^2+B2^2-B3^2, 2*(B2*B3+Bo*B1);...
       2*(B1*B3+Bo*B2), 2*(B2*B3-Bo*B1), Bo^2-B1^2-B2^2+B3^2;];

%Euler angles for 3-2-1 (Yaw-Pitch-Roll) Sequence
phi = atan2(DCM(2,3),DCM(3,3));
theta = -asin(DCM(1,3));
psi = atan2(DCM(1,2),DCM(1,1));

return;