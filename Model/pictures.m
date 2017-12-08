function [] = pictures()

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
       maxstroke RipForce1 RipForce2
   

[n m] = size(SRD);

figure(5)

for c = 1:8        
    Cx_rc = [Cargo(c,1); Cargo(c,2); Cargo(c,3)];
    Cx_e = Cx_rc;
    Cx(c,1) = Cx_e(1,1);
    Cy(c,1) = Cx_e(2,1);
    Cz(c,1) = Cx_e(3,1);
end
vertex_matrix_cargo = [Cx(2,1) Cy(2,1) Cz(2,1); Cx(8,1) Cy(8,1) Cz(8,1); Cx(6,1) Cy(6,1) Cz(6,1); Cx(4,1) Cy(4,1) Cz(4,1);... %bottom of box on the floor plane
                  Cx(1,1) Cy(1,1) Cz(1,1); Cx(7,1) Cy(7,1) Cz(7,1); Cx(5,1) Cy(5,1) Cz(5,1); Cx(3,1) Cy(3,1) Cz(3,1)];   %top of the box at a height                  
%Faces are created by combining the vertices to make the four corners of one side of the box 
%faces_matrix1 [ 1 2 6 5 ] = 
%[vertex_matrix1(1,1) vertex_matrix1(2,1) vertex_matrix1(6,1) vertex_matrix1(5,1)] 
faces_matrix_cargo = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8]; 
patch('Vertices',vertex_matrix_cargo,'Faces',faces_matrix_cargo,...
      'FaceVertexCData',hsv(6),'FaceColor','flat','LineWidth',1,'FaceAlpha',0.5)
hold

for i = 1:n
    if SRD(i,end) == 1
        line([SRD(i,1) SRD(i,4)],[SRD(i,2) SRD(i,5)],[SRD(i,3) SRD(i,6)],'Color','r','LineWidth',4)
    end
end

axis([-5 5 -1 3 -1 2]); 
axis normal;
grid on
view(45, 30);
pics = figure(5);
hold
% filenamesfig = ['fig' int2str(1) '.fig'];
% filenamesjpg = ['fig' int2str(1) '.jpg'];
% saveas(pics,filenamesfig)
% saveas(pics,filenamesjpg)