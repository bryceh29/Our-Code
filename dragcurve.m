% Purpose:
%		Simulate the steady flight of an autonomous aircraft.
%
% Desired Vehicle Parameters:
%       Max wingspan = 12 inches
%       Max weight = 1 lbs
%       Flight Time = 1 hr
%       
%
% Subroutines:
%		Atmosphere
%				Inputs: altitude, velocity, 
%				Outputs: windspeed, pressure, density, temperature, viscosity, Mach #, Reynolds #, q
%
%		Drag
%				Inputs: altitude, lift coeffient
%				Outputs: drag coefficient
%
%		Geometry
%				Fixed-weight: GPS, camera+beacon, control servos
%				Variable-weight: foam core, fiberglass skin
%				Discrete-weight: tail boom(s), tail (rudder & elevator)
%				Optional-weight: pressure probe
%		Velocity
%			a. Guess two velocities & call the drag subroutine
%			b. Use golden section (or similar) to find the best glide slope relative to the ground
%				 For small angles, the function to maximize is: (Vairspeed - Vinfinity) / [(L/D)*Vairspeed]
%			c. Return Vairspeed, Vgroundspeed, and L/D
%-----------------------------------------------------------------------------------------------------------

close all
clear all
disp('-----------------------------------------------------------------------------')
% Global variables
global DESIGN
global ALTITUDE
ALTITUDE = 2000;

%% INITIAL SETUP
% Define design variables
b_wing_in = 12;                 % wing span (in)
b_wing =  b_wing_in / 12;       % wing span (feet)
aspectratio = 2.25;
c_wing = b_wing/aspectratio;    % wing chord (feet)
CL_max = 0.8;                   % (doesn't affect the results, currently unused) practical limit (ARES drag polars don't accurately model higher CLs)
CL_stall = 0.65*0.9;           % limit for sake of sea level stall calc from X-Foil with MH81 and Re = 68e3
DESIGN = [b_wing, c_wing, CL_max];
design = DESIGN;

%% Type of output
flag1 = 0.0;	% show plots after every flight
flag2 = 0.0;	% show plots after a series of flights
flag_main = 1.0;    % 0 - turn off main program for subroutine testing 
                    % 1 - turn on main program
if flag_main == 1.0

            % OUTER LOOP - ITERATES OVER A RANGE OF DESIGN VARIABLES
            %WS = [0.5 : 0.25 : 5.0];
            %AR = [3 : 0.5 : 16];
            % for k = 1:1:length(AR);
            %   'Iteration # ', k
            %   W_by_S = WS(k);
            %   AR_w = AR(k);
            %   design = [W_by_S, AR_w, weight];
   
% Initialize variables
H_distance = 0.0;   
   
% BEGINNING OF FLIGHT
% Call geometry subroutine
dummy = 1.0;
[weight,S_w,c_w,AR_w] = geometry(dummy);
W_by_S = weight / S_w;

% PULL-OUT MANEUVER
alt_cr = 2000;		% Flight Altitude (ASL)
V_guess = 40;  % initial guess for best velocity (in ft/s)
                % Simple sim
                %g = 32.2;	% free-fall acceleration, ignoring drag (ft/s)
                %V_guess = 200;
                %[V] = velocity(Alt0,V_guess,design);
                %t = V / g;
                %d = g * t^2;
                %Alt1 = Alt0 - d; % Final altitude after pull-out & stabilization

            % Detailed sim
            % [V,track] = pullup2(weight);
            % Alt1 = ALTITUDE    % record pullout altitude

                    % GLIDE HOME
                    % %   First segment - length based on distance from current altitude to nearest 1000ft
                    %     Init_dist = 0.0;	% initialize distance traveled

                    % 	dh = mod(ALTITUDE,1000); % vertical distance of first segment
                    %     [V_optimal(1), V_wind(1), L_by_D(1)] = velocity(V_guess);
                    %     distance(1) = L_by_D(1) * ( (V_optimal(1) - V_wind(1)) / V_optimal(1) ) * dh;
                    %     H_distance(1) = Init_dist + distance;	% Track the horizontal distance traveled
                    %     V_guess = V_optimal(1);	% set next guess to current optimum
                    %     Alt2 = Alt1 - dh;   % record altitude at end of first segment
                       %%%%%%Want to change for loop to SINGLE calculation, none of the
                       %%%%%%parameters in this respect change over time
                    % Remaining segments are taken in increments of 1000ft altitude loss
                    % stepsize = 1000;	% step size (ft)

% Call velocity subroutine
[V_optimal, V_wind, L_by_D] = velocity(V_guess); ... (V_optimal in ft/s)
V_opt=V_optimal*0.592484 ... (kts)
                    %    distance = L_by_D * ( (V_optimal - V_wind) / V_optimal ) 1000;
                    %    H_distance = H_distance;... + distance;
                    %    V_guess = V_optimal;	% set next guess to current optimum

% FLIGHT TIME
[P_R, t_flight, range] = Pow_f(V_optimal,weight);
    
Total_distance = range        % total distance traveled (nmi)
                    % H_distance = (V_optimal - V_wind)*T_max; % Distance Traveled (ft)
                    % H_distance = H_distance / 6076.1155;	% convert distances from ft to nmi

ALTITUDE = 0;
[windspeed, P, rho, T, mu, nu, mach, REL, q] = atmosphere(V_optimal); 
V_stall = 0.5924838 * sqrt( (2*W_by_S) / (rho*CL_stall))  % (kts)
[windspeed, P, rho, T, mu, nu, mach, REL_stall, q] = atmosphere(V_stall);
Re = REL*c_w;
Re_stall = REL_stall*c_w;
disp('Re at cruise = '); disp(Re);
disp('Re at Stall = '); disp(Re_stall);

% % Plot the wing area:
% [D_zero,Cd,Cdp,Cdi,CL,V_min] = drag(V_optimal);
% visualize(CL, V_opt, weight, b_wing_in, rho)

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
    
if flag1 == 1.0
%     Altitude = [Alt1, Alt2: -stepsize: 1000];
%     alt = Altitude / 1000;
% 
% 	figure(1)
% 	plot(0.5924838*V_wind,alt)
% 	ylabel('Altitude (k*ft)')
% 	xlabel('Windspeed (kts)')
% 	title('Altitude vs Windspeed')

% 	figure(2)
%     x2 = [track(:,3)',V_optimal];
%     x2 = 0.5924838 * x2; % convert from ft/s to kts
%     y2 = [track(:,2)'/1000,alt];
% 	%plot(V_opt1,alt)
%     plot(x2,y2)
% 	xlabel('Optimal Velocity (kts)')
% 	ylabel('Altitude (k*ft)')
% 	title('Altitude vs Velocity')

% 	figure(3)
% 	for i = 1:length(alt)
%         ALTITUDE = Altitude(i);
%         [windspeed, P, rho(i), T, mu, nu(i), Mach(i), RE(i), q(i)] = atmosphere(V_optimal(i));
%     end
%     x3 = [track(:,7)',Mach];
%     y3 = [track(:,2)'/1000,alt];
% 	plot(x3,y3)
%     %plot(Mach,alt)
% 	xlabel('Mach number')
% 	ylabel('Altitude (k*ft)')
% 	title('Altitude vs Mach #')

% 	figure(4)
% 	plot(L_by_D,alt)
% 	xlabel('Lift-to-Drag Ratio')
% 	ylabel('Altitude (k*ft)')
% 	title('Altitude vs (L/D)')
   
%    figure(5)
%    [weight,S_w,c_wing] = geometry(dummy);
%    CL = weight ./ (q * S_w);
%    plot(CL,alt)
%    xlabel('Lift coefficient')
%    ylabel('Altitude (k*ft)')
%    title('Altitude vs CL')
%    a0 = 2*pi;  % lift curve slope for an ideal airfoil
%    e = 0.90;	% approx value for span efficiency
%    tau = (1/e)-1;	% see Anderson, pg. 374
%    alpha = a0 / ( 1 + (a0/(pi*AR_w))*(1+tau) );  %AoA corrected for finite wing effect
%    % Also, need to correct for compressibility effects
   
   figure(6)
   plot(H_distance,alt_cr,'o')
   xlabel('Ground Distance (nmi)')
   ylabel('Altitude (k*ft)')
   title('Altitude vs Distance Traveled')
   grid
   
   
%    figure(7)
%    angle1 = (dh / distance(1)) * (180/pi);
%    angle = (stepsize ./ distance(2:end)) * (180/pi); % based on small angle approx
%    angle = [angle1 angle];
%    plot(angle,alt)
%    xlabel('Glide Slope Angle (deg)')
%    ylabel('Altitude (k*ft)')
%    title('Altitude vs Glide Slope Angle')

%    figure(8)
%    Re_wing = RE * c_wing;
%     
%     x8 = [track(:,8)',Re_wing];
%     y8 = [track(:,2)'/1000,alt];
%    %plot(Re_wing,alt)
%    plot(x8,y8)
%    xlabel('Reynolds Number - based on wing aero chord')
%    ylabel('Altitude (k*ft)')
%    title('Altitude vs Reynolds number')
   
   figure(9)
   distance = H_distance * 6076.1155;	% convert distance from nmi to ft
   init_dist = 0;
   for i = 1:length(alt)
      if i == 1
         time(i) = (distance(i) - init_dist) / V_optimal(i);
      else
         time(i) = time(i-1) + (distance(i) - distance(i-1)) / V_optimal(i);
      end
   end
   time = time / 60;	% convert time from sec to mi
	plot(time,Re_wing)
	xlabel('Time (min)')
	ylabel('Reynolds Number - based on wing aero chord')
   title('Reynolds number vs Time')
   
   figure(10)
   plot(time,CL)
   xlabel('Time (min)')
   ylabel('Lift Coefficient')
   title('CL vs Time')   
end

if flag2 == 1.0
   MaxDist(k) = H_distance(end);
   Vmax(k) = max(V_optimal);
   CL_max = 1.0;	% assume a value
   [windspeed, P, rho, T, mu, nu, mach, Re, q] = atmosphere(0,V_optimal(end)); 
   V_stall(k) = 0.5924838 * sqrt( (2*WS(k)) / (rho*CL_max) );   % (kts)

   if k == length(WS)
      figure(1)
      plot(WS,MaxDist)
      xlabel('Wing Loading (psf)')
      ylabel('Distance traveled (nmi)')
      title('Distance Traveled vs Wing Loading')
      
      figure(2)
      plot(WS,Vmax)
      xlabel('Wing Loading (psf)')
      ylabel('Maximum velocity (ft/s)')
      title('Max Velocity vs Wing Loading')
      
      figure(3)
      plot(WS,V_stall)
      xlabel('Wing Loading (psf)')
      ylabel('Stall Velocity at Sea Level (kts)')
      title('Stall Velocity vs Wing Loading')
   end
end

%end	% END OF OUTSIDE LOOP
end	% provides a means of turning off the main loop for subroutine testing
%---------------------------------------------------------------------
%---------------------------------------------------------------------

% % Code for calculating drift distance of balloon & payload during ascent
% drift_test = 0.0;
% if drift_test == 1.0
%     A = [800 1000 1200];
%     for i = 1:1:length(A);
%         V = 0;  % balloon & payload drift with the wind
%         ascent = A(i) / 60;   % ascent rate (ft/sec)
%         dh = 1000;    % stepsize (ft)
%         step = 1;   % initialize counter
%         for h = 0 : dh : 100000
%             ALTITUDE = h;
%             [V_wind] = atmosphere(V);
%             time(step) = dh / ascent;   % time to climb (sec)
%             if step == 1
%                 drift(step) = V_wind * time(step);  % drift distance
%             else
%                drift(step) = drift(step-1) + V_wind * time(step);  % drift distance
%             end
%            step = step + 1;    % increment counter
%         end
%         Drift(i,:) = drift / 6076;   % convert from ft to nmi
%     end
%     DriftDistance = [Drift(1,end),Drift(2,end),Drift(3,end)]
%     H = [0 : dh : 100000] / 1000;
%     plot(Drift(1,:),H,  Drift(2,:),H,  Drift(3,:),H)
%     legend('Ascent Rate = 800 ft/min','Ascent Rate = 1000 ft/min','Ascent Rate = 1200 ft/min')
%     xlabel('Drift Distance (nmi)')
%     ylabel('Altitude (k*ft)')
%     title('Altitude vs Drift Distance')
%     grid on
% end
%------------------------------------------------------------------

% Code for testing the atmosphere subroutine
%		Testing complete - results agree to within 1% of those listed in 
%		the standard atmosphere table in Anderson's "Intro to Flight," 4th Ed.
A_test = 0.0;
if A_test == 1.0
	vel = 100;
	for altitude = 1000 : 1000 : 60000
      [windspeed, P, rho, T, mu, nu, mach, Re, q] = atmosphere(altitude,vel);
      altitude
      T
      rho
      P
   end
end

%---------------------------------------------------------------------
% Code for testing the velocity subroutine
V_test = 0.0;
if V_test == 1.0
   altitude = 40000;
   V_guess = 200;
	[V_optimal, V_wind, L_by_D] = velocity(altitude,V_guess,design)
   
   V1 = 120: 1.0 : 150;
	for i = 1:length(V1)
      [D1(i),Cd,Cdp,Cdi,CL] = drag(altitude,V1(i),design);
      obj(i) = V1(i) / ( V1(i) - V_wind ) * D1(i);
	   [windspeed, P, rho, T, mu, nu, mach, Re, q] = atmosphere(altitude,V1(i));
      V1(i);
      Re * 0.42817442;  % Reynolds number based on wing chord (for W/S=3, W=4.4, AR=8)
   end
   
   V2 = 50: 1.0 : 140;
   for j = 1:length(V2) 
      [D2(j),Cd,Cdp,Cdi,CL] = drag(altitude,V2(j));
	end
      
	figure(1)
   plot(V2,D2)
   title('Drag vs. Airspeed')
   figure(2)
   plot(V1,obj)
   title('Objective vs. Airspeed')
end
%-------------------------------------------------------------------

% Code for testing the drag subroutine
D_test = 0.0;
if D_test == 1.0

ALTITUDE = 90000;

for V = 60:10:720
   V;
   [D_total(V/10-5),Cd(V/10-5),Cdp(V/10-5),Cdi(V/10-5),CL(V/10-5)] = drag(V);
end

V = [60:10:720];
A = V ./ (6.0 ./ D_total);
B = V - 29;
Obj = A ./ B;

figure(1)
plot(V,Obj)
xlabel('Velocity')
ylabel('Objective')

figure(2)
plot(V,(6./D_total))
xlabel('Velocity')
ylabel('L/D')

figure(3)
plot(V,Cdi,V,Cdp,V,Cd)
xlabel('Velocity')
ylabel('L/D')
legend('Cdi','Cdp','Cd')
end
%------------------------------------------------------------------