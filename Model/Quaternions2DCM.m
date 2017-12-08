function DCM = Quaternions2DCM(Bo,B1,B2,B3,a)
%Quaternions -> Direction Cosine Matrix

%Earth to body (i.e. cargo,platform) CS
DCMeb = [Bo^2+B1^2-B2^2-B3^2, 2*(B1*B2+Bo*B3), 2*(B1*B3-Bo*B2);...
       2*(B1*B2-Bo*B3), Bo^2-B1^2+B2^2-B3^2, 2*(B2*B3+Bo*B1);...
       2*(B1*B3+Bo*B2), 2*(B2*B3-Bo*B1), Bo^2-B1^2-B2^2+B3^2;];

DCMbe = DCMeb';

if a == 1
    DCM = DCMbe; %Rotations from Cargo or Platform to Earth fixed cs
    
elseif a == 2
    DCM = DCMeb; %Rotate from Earth fixed to Platform CS        
end

return;