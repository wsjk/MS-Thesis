    %Run SRDmovie.m to create a movie of the results
function SRDmovie(az,el)

global a b h h2 W L g...
       smax smax2 RipForce K1 K2 K3 C...
       x1 x2 y1 y2 z1 z2...
       Boep B1ep B2ep B3ep Boec B1ec B2ec B3ec...
       SRD Cargo Platform Ground timestep...

[n,g] = size(SRD); %n = total number of SRDs 
[p,q] = size(Cargo); 
cgc = Cargo(p,1:3); %coord vector of cargo CG [x y z]
corners = p-1; %corners = number of corners of cargo (8 for a box)
[m,q] = size(x1); %Dimensions of u1 vector (i.e. how many data points)
% G = moviein(m);
G = moviein(350);

FloorX_e = zeros(n,1);
FloorY_e = zeros(n,1);
FloorZ_e = zeros(n,1);
SRDX_e = zeros(n,1);
SRDY_e = zeros(n,1);
SRDZ_e = zeros(n,1);
Cx = zeros(corners,1);
Cy = zeros(corners,1);
Cz = zeros(corners,1);
Px = zeros(8,1);
Py = zeros(8,1);
Pz = zeros(8,1);
sets =1;
for i = 1:200:m
    newplot

    DCMpe = Quaternions2DCM(Boep(i,1),B1ep(i,1),B2ep(i,1),B3ep(i,1),1); %DCM, Platform -> Earth
    DCMep = Quaternions2DCM(Boep(i,1),B1ep(i,1),B2ep(i,1),B3ep(i,1),2); 
    DCMce = Quaternions2DCM(Boec(i,1),B1ec(i,1),B2ec(i,1),B3ec(i,1),1); 
    DCMec = Quaternions2DCM(Boec(i,1),B1ec(i,1),B2ec(i,1),B3ec(i,1),2); 
    % [x2 y2 z2]%Transform translations from earth fixed to cargo 
    for j = 1:n %for the number of SRDs
        x_rc1 = DCMpe * [SRD(j,1);SRD(j,2);SRD(j,3)];
        x_e1 = [x1(i,1)+x_rc1(1,1); y1(i,1)+x_rc1(2,1); z1(i,1)+x_rc1(3,1)];
        FloorX_e(j,1) = x_e1(1,1);
        FloorY_e(j,1) = x_e1(2,1);
        FloorZ_e(j,1) = x_e1(3,1);
        x_rc2 = DCMce * [SRD(j,4);SRD(j,5);SRD(j,6)];
        x_e2 = [x2(i,1)+x_rc2(1,1); y2(i,1)+x_rc2(2,1); z2(i,1)+x_rc2(3,1)];
        SRDX_e(j,1) = x_e2(1,1);
        SRDY_e(j,1) = x_e2(2,1);
        SRDZ_e(j,1) = x_e2(3,1);
        % Draw black line to represent SRD
        line([FloorX_e(j,1) SRDX_e(j,1)],[FloorY_e(j,1) SRDY_e(j,1)],[FloorZ_e(j,1) SRDZ_e(j,1)]...
              ,'Marker','.','LineStyle','-','Linewidth',3,'Color','Red')
    end
    %Animation of cargo box
    for c = 1:corners        
        Cx_rc = DCMce * [Cargo(c,1); Cargo(c,2); Cargo(c,3)];
        Cx_e = [x2(i,1)+Cx_rc(1,1); y2(i,1)+Cx_rc(2,1); z2(i,1)+Cx_rc(3,1)]; %Coordinate vector of SRD cargo attachment point relative to platform cs
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

    %Four corners of pallet
    for p = 1:8
        Px_rp = DCMpe * [Platform(p,1); Platform(p,2); Platform(p,3)];
        Px_e = [x1(i,1)+Px_rp(1,1); y1(i,1)+Px_rp(2,1); z1(i,1)+Px_rp(3,1)];
        Px(p,1) = Px_e(1,1);
        Py(p,1) = Px_e(2,1);
        Pz(p,1) = Px_e(3,1);
    end
    vertex_matrix_platform = [Px(2,1) Py(2,1) Pz(2,1); Px(8,1) Py(8,1) Pz(8,1); Px(6,1) Py(6,1) Pz(6,1); Px(4,1) Py(4,1) Pz(4,1);... %Coordinates of corners on bottom of box on the floor plane
                      Px(1,1) Py(1,1) Pz(1,1); Px(7,1) Py(7,1) Pz(7,1); Px(5,1) Py(5,1) Pz(5,1); Px(3,1) Py(3,1) Pz(3,1)];   %Coordinates of corners on top of the box at a height "h"3                  
    %Faces are created by combining the vertices to make the four corners of one side of the box 
    %faces_matrix1 [ 1 2 6 5 ] = 
    %[vertex_matrix1(1,1) vertex_matrix1(2,1) vertex_matrix1(6,1) vertex_matrix1(5,1)] 
    faces_matrix_platform = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8]; 
    patch('Vertices',vertex_matrix_platform,'Faces',faces_matrix_platform,...
          'FaceVertexCData',hsv(8),'FaceColor','Blue','FaceAlpha',1)
      
    title(['t = ',num2str(timestep*i)],'FontSize',18)
    
%     for p = 1:8
%         Gx_e = [Ground(p,1); Ground(p,2); Ground(p,3)];
%         Gx(p,1) = Gx_e(1,1);
%         Gy(p,1) = Gx_e(2,1);
%         Gz(p,1) = Gx_e(3,1);
%     end
%     vertex_matrix_platform = [Gx(2,1) Gy(2,1) Gz(2,1); Gx(8,1) Gy(8,1) Gz(8,1); Gx(6,1) Gy(6,1) Gz(6,1); Gx(4,1) Gy(4,1) Gz(4,1);... %Coordinates of corners on bottom of box on the floor plane
%                   Gx(1,1) Gy(1,1) Gz(1,1); Gx(7,1) Gy(7,1) Gz(7,1); Gx(5,1) Gy(5,1) Gz(5,1); Gx(3,1) Gy(3,1) Gz(3,1)];   %Coordinates of corners on top of the box at a height "h"3                  
%     %Faces are created by combining the vertices to make the four corners of one side of the box 
%     %faces_matrix1 [ 1 2 6 5 ] = 
%     %[vertex_matrix1(1,1) vertex_matrix1(2,1) vertex_matrix1(6,1) vertex_matrix1(5,1)] 
%     faces_matrix_platform = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8]; 
%     patch('Vertices',vertex_matrix_platform,'Faces',faces_matrix_platform,...
%           'FaceVertexCData',hsv(8),'FaceColor','Black','FaceAlpha',.5)
      
      
    axis([-15 10 -15 10 -4 10]); 
    %     axis([-5 25 -5 25 -4 10]); 
    axis normal;
    grid on
    view(az, el);
    set(gca,'FontSize',18);
    set(gcf,'Position', [360 50 960 600]); %big screen
%     set(gcf,'Position', [110 100 560 550]); %smaller screen
    G(sets) = getframe(gcf);
    sets = sets+1;
end

% movie(G,1,64);
% movie2avi(G, 'SRD_movie', 'fps',250,'quality',90); 

mpgwrite(G,[],'my_truecolor_movie.mpg');