% Nathan Curry
% Dev. Spring 2013
% UAS Project

function visualize(CL_optimal, V, W, b, density, V_stall);
%% Description:
% This program plots the wing shape specified in the main function, and
% calculates an aspect ratio for comparison against the input AR.
% 
% 
% Inputs: 
%   CL = coefficient of lift during cruise
%   V  = Cruise velocity in knots
%   W  = weight in pound force
%   b  = wingspan in inches
%   density = air density in slugs/ft^3
% 
% 
% 
% 
global DesignStruct
aspectratio = DesignStruct.AR;  % get aspect ratio defined in 'main.m'
%% Main plot
    size = 60;
    V_flagg = linspace(V_stall,70,size);
    
    % Initialize matrices for plots:
    drag_total = zeros(1,size);
    drag_i = zeros(1,size);
    drag_p=zeros(1,size);
    time =zeros(1,size);
    distance = zeros(1,size);
    for n = 1:size   % [ft/s]
        [D_zero,Cd,Cdp,Cdi,CL,V_min,D_i,D_p] = drag(V_flagg(n)/0.592484);
        [P_R, t_flight, range, E] = Pow_f(V_flagg(n)/0.592484,W);
        drag_total(n) = D_zero;
        drag_i(n) = D_i;
        drag_p(n) = D_p;
        time(n) = t_flight;
        distance(n) = range;
    end
    set(figure,'units','normalized','outerposition',[.1 .1 .75 .75]); % reset plot window size
    plot(V_flagg,drag_total,'b-',V_flagg,drag_i,'b:',V_flagg,drag_p,'b--',V_flagg,time/10,'g',V_flagg,distance/400,'m')
    title(['Drag, Endurance, and Range for AR = ' num2str(aspectratio) ', Efficiency = ' num2str(E)])
    xlabel('Velocity, knots')
    ylabel('Drag, lbf')
    line([V_flagg(1) V_flagg(size)],[0.11 0.11],'LineStyle',':','Color','g')
    line([V_stall V_stall],[0 max(time)/10],'LineStyle','-','Color','r')
    legend('Total Drag','Induced Drag','Parasite Drag','Endurance, hr/10','Range, nmi/400','location','EastOutside')
    grid
    text(V_stall,-0.01,'Stall','HorizontalAlignment','Center','Color','r')
    text(72,0.11,'Target Endurance','Color','g')

%% Create general plots:
set(figure,'units','normalized','outerposition',[.1 .1 .75 .75]); % reset plot window size
% velocity:
vel_knots=15:1:40;...(knots)
vel_us=vel_knots*1.68781;... (ft/s)

% Wing Surface Area Required:
S=2*W./(density*CL_optimal.*vel_us.^2); %(ft^2)
S_in=S*144;                     % (in^2)
subplot(2,2,1);
hold on
plot(vel_knots,S_in,'k');
title('Wing Surface Area vs. Cruise Velocity');
xlabel('Velocity (knots)')
ylabel('Area (in^2)')
grid

% Predict wing shape: 

% Set Wingspan:
b1=72;... (in)
b2=74; 
b3=96;
b4=108;
% Aspect Ratio:
AR1=b1^2./S_in;
AR2=b2^2./S_in;
AR3=b3^2./S_in;
AR4=b4^2./S_in;
subplot(2,2,2);
plot(vel_knots,AR1,'k',vel_knots,AR2,'r',vel_knots,AR3,'b',vel_knots,AR4,'g')
legend('b=60"','b=72"','b=84"','b=96"','Location','EastOutside')
title('Aspect Ratio vs. Cruise Velocity for Various Wingspans');
xlabel('Velocity (knots)')
ylabel('AR')
grid


%% Plot Wing Top view for cruise:

        % Set variables for wing to be plotted:
        
        theta=0; ... Leading edge sweep angle.  (deg)
        phi=-5; ... Trailing edge sweep angle.  (deg)
        vel=V*1.68781; ...  (ft/s) V must be in domain of vel_knots and is then rounded to nearest integer;
           
% Calculate Wing Area:
S_wing=2*W/(density*CL_optimal*vel^2);... (ft^2)
S_wing_in=S_wing*144;... (in^2)
h=(S_wing_in/b)-b/4*(tand(theta)-tand(phi))
wing_X=[0 b/2 b b b/2 0 0];
wing_Y=[0 b/2*tand(phi) 0 h h+b/2*tand(theta) h 0];
AR=b^2/S_wing_in;
subplot(2,2,3);
plot(wing_X,wing_Y)
text(2,h/2, 'AR = ')
text(10,h/2, num2str(AR))
axis([-2,b+2,-2,h+2]);
axis equal;
title('Wing Planform, Velocity = V_c_r_u_i_s_e knots (top view)')
xlabel('inches')
ylabel('inches')

end


%% Pullup manuever plots (not been verified in new location(here) they were originally in 'main' but were removed for space
%% % end

% Compute maximum Mach number (from pullup routine)
% M_max = max(track(:,7))

% END OF FLIGHT ---------------------------------------------------------

% GENERATE PRETTY PICTURES

% % % % Drag vs. Velocity
% % % figure(1)
% % % n=1;
% % % 
% % % for Vplot=10:1:400;
% % %    [D_total,Cd,Cdp,Cdi,CL,V_min] = drag(Vplot);
% % %    y1(n)=Cdp;
% % %    y2(n)=Cdi;
% % %    Vplot(n)=Vplot;
% % %    n=n+1;
% % % end
% % % 
% % % plot(Vplot,y1,Vplot,y2)
% % %     legend('Cdi','Cdp')

% if flag2 == 1.0
%    MaxDist(k) = H_distance(end);
%    Vmax(k) = max(V_optimal);
%    CL_max = 1.0;	% assume a value
%    [windspeed, P, rho, T, mu, nu, mach, Re, q] = atmosphere(0,V_optimal(end)); 
%    V_stall(k) = 0.5924838 * sqrt( (2*WS(k)) / (rho*CL_max) );   % (kts)
% 
%    if k == length(WS)
%       figure(1)
%       plot(WS,MaxDist)
%       xlabel('Wing Loading (psf)')
%       ylabel('Distance traveled (nmi)')
%       title('Distance Traveled vs Wing Loading')
%       
%       figure(2)
%       plot(WS,Vmax)
%       xlabel('Wing Loading (psf)')
%       ylabel('Maximum velocity (ft/s)')
%       title('Max Velocity vs Wing Loading')
%       
%       figure(3)
%       plot(WS,V_stall)
%       xlabel('Wing Loading (psf)')
%       ylabel('Stall Velocity at Sea Level (kts)')
%       title('Stall Velocity vs Wing Loading')
%    end
% end