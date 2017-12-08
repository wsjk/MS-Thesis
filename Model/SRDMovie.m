    %Run SRDmovie.m to create a movie of the results
function SRDmovie(az,el,frames)

global cw cl h h2 PW PL g...
       smax smax2 RipForce K1 K2 K3 C...
       x1 x2 y1 y2 z1 z2 t u2 u1...
       Boep B1ep B2ep B3ep Boec B1ec B2ec B3ec...
       GRID Cargo Platform Ground timestep ForceTotal s0...
       offsetx offsety offsetz

[n,g] = size(GRID); %n = total number of restraints 
[p,q] = size(Cargo); 
corners = p; %corners = number of corners of cargo (8 for a box)
[m,q] = size(x1); %Dimensions of u1 vector (i.e. how many data points)
% G = moviein(m);
G = moviein(600);


FloorX_e = zeros(n,1);
FloorY_e = zeros(n,1);
FloorZ_e = zeros(n,1);
GRIDX_e = zeros(n,1);
GRIDY_e = zeros(n,1);
GRIDZ_e = zeros(n,1);
Cx = zeros(corners,1);
Cy = zeros(corners,1);
Cz = zeros(corners,1);
Px = zeros(8,1);
Py = zeros(8,1);
Pz = zeros(8,1);
sets = 1;

for i = 1:frames:m
    newplot    

    DCMpe = Quaternions2DCM(Boep(i,1),B1ep(i,1),B2ep(i,1),B3ep(i,1),1); %DCM, Platform -> Earth
    DCMep = Quaternions2DCM(Boep(i,1),B1ep(i,1),B2ep(i,1),B3ep(i,1),2); 
    DCMce = Quaternions2DCM(Boec(i,1),B1ec(i,1),B2ec(i,1),B3ec(i,1),1); 
    DCMec = Quaternions2DCM(Boec(i,1),B1ec(i,1),B2ec(i,1),B3ec(i,1),2); 
    % [x2 y2 z2]%Transform translations from earth fixed to cargo 
    for j = 1:n %for the number of restraints
        if GRID(j,end) == 1
            x_rc1 = DCMpe * [GRID(j,1);GRID(j,2);GRID(j,3)];
            x_e1 = [x1(i,1)+x_rc1(1,1); y1(i,1)+x_rc1(2,1); z1(i,1)+x_rc1(3,1)];
            FloorX_e(j,1) = x_e1(1,1);
            FloorY_e(j,1) = x_e1(2,1);
            FloorZ_e(j,1) = x_e1(3,1);
            x_rc2 = DCMce * [GRID(j,4);GRID(j,5);GRID(j,6)];
            x_e2 = [x2(i,1)+x_rc2(1,1); y2(i,1)+x_rc2(2,1); z2(i,1)+x_rc2(3,1)];
            GRIDX_e(j,1) = x_e2(1,1);
            GRIDY_e(j,1) = x_e2(2,1);
            GRIDZ_e(j,1) = x_e2(3,1);
            % Draw red line to represent restraint
            line([FloorX_e(j,1) GRIDX_e(j,1)],[FloorY_e(j,1) GRIDY_e(j,1)],[FloorZ_e(j,1) GRIDZ_e(j,1)]...
                  ,'Marker','.','LineStyle','-','Linewidth',3,'Color','Red')
            dx2 = x_e2(1,1)-x_e1(1,1);
            dy2 = x_e2(2,1)-x_e1(2,1);
            dz2 = x_e2(3,1)-x_e1(3,1);
            S(j,1) = sqrt(dx2^2 + dy2^2 + dz2^2) - s0(j,1);
            if S(j,1) >= smax(j,1)
                line([FloorX_e(j,1) GRIDX_e(j,1)],[FloorY_e(j,1) GRIDY_e(j,1)],[FloorZ_e(j,1) GRIDZ_e(j,1)]...
                  ,'Marker','.','LineStyle','-','Linewidth',3,'Color','Black')
            end
        end
    end
    
    line([x1(i,1) x2(i,1)+offsetx],[y1(i,1) y2(i,1)+offsety],[z1(i,1) z2(i,1)+offsetz]...
        ,'Marker','.','LineStyle','-','Linewidth',3,'Color','Black')
    
    %Animation of cargo box
    for c = 1:corners        
        Cx_rc = DCMce * [Cargo(c,1); Cargo(c,2); Cargo(c,3)];
        Cx_e = [x2(i,1)+Cx_rc(1,1); y2(i,1)+Cx_rc(2,1); z2(i,1)+Cx_rc(3,1)]; %Coordinate vector of restraint cargo attachment point relative to platform cs
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
          'FaceVertexCData',hsv(8),'FaceColor','Blue','FaceAlpha',.4)
    
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
      
%     axis([-15 10 -15 10 -4 10]); 
    
%     axis([-8 5 -3 3 -1 5]); 
    axis([-5 4 -5 4 0 3]); 
%     axis([-4 4 -4 4 0 3]); 
    axis normal;
    grid on
    view(az, el);
    xlabel('Longitudinal [m]','fontsize',25,'fontweight','b')
    ylabel('Lateral [m]','fontsize',25,'fontweight','b')
    zlabel('Vertical [m]','fontsize',25,'fontweight','b')
    title(['t = ',num2str(timestep*i)],'FontSize',20,'FontWeight','bold')
    
    set(gcf,'Position', [360 60 1060 700]); %big screen
%     set(gcf,'Position', [110 100 560 550]); %smaller screen
    set(gca,'FontSize',25,'FontWeight','bold');
    G(sets) = getframe(gcf);
    sets = sets+1;
end

% movie(G,1,64);
% movie2avi(G, 'SRD_movie', 'fps',250,'quality',90); 

mpgwrite(G,[],'cargo.mpg');