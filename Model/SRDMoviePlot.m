    %Run SRDmovie.m to create a movie of the results
function SRDMoviePlot(frames)

global cw cl h h2 PW PL g...
       smax smax2 RipForce K1 K2 K3 C...
       x1 x2 y1 y2 z1 z2 t u2 u1...
       Boep B1ep B2ep B3ep Boec B1ec B2ec B3ec...
       SRD Cargo Platform Ground timestep ForceTotal...

[n,g] = size(SRD); %n = total number of SRDs 
[p,q] = size(Cargo); 
[m,q] = size(x1); %Dimensions of u1 vector (i.e. how many data points)
% G = moviein(m);
G = moviein(600);

sets = 1;

% for i = 1:frames:m
for i = 1:frames:2801
    
    Plot1 = subplot(3,1,1); plot(t(1:i,1),ForceTotal(1:i,:),'LineWidth',5);
    
%     axis([0  max(t)  0 max(ForceTotal(:,1))*1.10])
axis([0  .276  0 max(ForceTotal(:,1))*1.10])

    set(gca,'FontSize',20,'FontWeight','bold');
%     title(['t = ',num2str(timestep*i)],'FontSize',20,'FontWeight','bold')
    xlabel('Time [s]','fontsize',20,'fontweight','b')
    ylabel('Force [N]','fontsize',20,'fontweight','b')
    Plot2 = subplot(3,1,2);plot(t(1:i,1),(x2(1:i,1)-x1(1:i,1)),'LineWidth',5);
    
%     axis(Plot2,[0  max(t)  0 max(x2-x1)*1.10])
axis(Plot2,[0 .276  0 max(x2-x1)*1.10])

    set(gca,'FontSize',20,'FontWeight','bold');
    xlabel('Time [s]','fontsize',20,'fontweight','b')
    ylabel('Displacement [m]','fontsize',20,'fontweight','b')
    Plot3 = subplot(3,1,3);plot(t(1:i,1),(u2(1:i,1)-u1(1:i,1)),'LineWidth',5);
    
%     axis(Plot3,[0  max(t)  0 max(u2-u1)*1.10])
axis(Plot3,[0  .276  0 max(u2-u1)*1.10])

    set(gca,'FontSize',20,'FontWeight','bold');
    xlabel('Time [s]','fontsize',20,'fontweight','b')
    ylabel('Velocity [m/s]','fontsize',20,'fontweight','b')
    
    set(gcf,'Position', [360 60 1060 700]); %big screen
%     set(gcf,'Position', [110 100 560 550]); %smaller screen
    set(gca,'FontSize',20,'FontWeight','bold');
    G(sets) = getframe(gcf);
    sets = sets+1;
end

mpgwrite(G,[],'plots.mpg');