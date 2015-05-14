
% Purpose:
%		Simulate the steady flight of an autonomous aircraft.
%
% Desired Vehicle Parameters:
%       Max wingspan = 120 inches
%       Max weight = 5 lbs
%       Flight Time = inf hr
%       
%
% Subroutines:
%	 	Atmosphere
%				Inputs: altitude, velocity, Outputs: windspeed, pressure,
%				density, temperature, viscosity, Mach #, Reynolds #, q
%
%		Drag
%				Inputs: altitude, lift coeffient
%				Outputs: drag coefficient
%
%		Geometry
%				Fixed-weight: GPS, Autopilot, control servos
%				Variable-weight: foam core, fiberglass skin
%				Discrete-weight: tail boom(s), tail (rudder & elevator)
%				Optional-weight: payload
% 
%       Stabilty
%               Optimally determine longintudinal configuration of
%               components along x-axis (through nose) to balance pitching
%               moment
%        
%		Velocity
%			a. Guess two velocities & call the drag subroutine
%			b. Use golden section (or similar) to find the best glide slope 
%              relative to the ground. For small angles, the function to
%              maximize is: (Vairspeed - Vinfinity) / [(L/D)*Vairspeed]
%			c. Return Vairspeed, Vgroundspeed, and L/D
%               (V_optimal in ft/s)
%               (V_opt in kts)
%--------------------------------------------------------------------------

close all
clear all
clc
format compact
disp('-------------------------------------------------------------------')
% Global variables
global DESIGN ALTITUDE DesignStruct
ALTITUDE = 300;

%% INITIAL SETUP ==========================================================
% Define design variables
b_wing_in = 96;                 % wing span (in)
b_wing =  b_wing_in / 12;       % wing span (feet)
taper_wing = 1;                 % wing taper
aspectratio = 8;               % for aspectratio = 1.75:0.5:3.25
% Specify initial guesses
alt_cr = 300;                   % Flight Altitude (ASL) [ft]
V_guess = 50;                   % initial guess for best velocity (in ft/s)
% Define optimization method
%   1 - Newton-based optimizer
%   2 - Safe-guarded polynomial fitting
method = 2;
c_wing = b_wing/aspectratio;    % wing chord (feet)
CL_max = 0.8;                   % (doesn't affect the results, currently unused) 
                                % practical limit (ARES drag polars don't accurately model higher CLs)
CL_stall = CL_max*0.9;          % limit for sake of sea level stall. 0.65 
                                % is from X-Foil with MH81 and Re = 68e3 and 0.9 is guessed factor of safety
DESIGN = [b_wing, c_wing, CL_max, aspectratio];
DesignStruct.AR = aspectratio;
design = DESIGN;

% Airfoil selection
global AIRFOIL AIRFOIL_WL
AIRFOIL = 7; ... Airfoil of main wing
AIRFOIL_WL = 7; ... Airfoil of winglet
%   0 - NACA 5310 airfoil
%   1 - ARES airfoil with quadratic drag polar representation
% 	2 - SD7037 with single quadratic drag polar representation
%	3 - SD7037 with split quadratic drag polar representation
%   4 - ARES airfoil
%   5 - MH81 flying wing airfoil
%   6 - PSU 94-097 winglet airfoil
%   7 - E 205 RC glider airfoil 

% TESTS AND DIAGNOSTICS - see end of main for test loops =========\

% Feed into functions when test toggle input is unnecessary 
% (i.e. main routine calls - BEGINNING OF FLIGHT section) 
global dummy dummy2
dummy = 0;      
dummy2 = 0;

% Define Test flags
    % 0 - turn off main program for subroutine testing 
    % 1 - turn on main program
A_flag = 0.0;   % Atmosphere test routing
V_flag = 0.0;   % Flag test routine 
D_flag = 0.0;

% =========================================================================
%% Type of output
flag1 = 1.0;	% show plots after every flight
flag2 = 0.0;	% show plots after a series of flights
flag3 = 1.0;    % show drag plot and wing area visualization
flag_main = 1.0;    % 0 - turn off main program for subroutine testing 
                    % 1 - turn on main program
if flag_main == 1.0

            % OUTER LOOP - ITERATES OVER A RANGE OF DESIGN VARIABLES
            % WS = [0.5 : 0.25 : 5.0];
            % AR = [3 : 0.5 : 16];
            % for k = 1:1:length(AR);
            %   'Iteration # ', k
            %   W_by_S = WS(k);
            %   AR_w = AR(k);
            %   design = [W_by_S, AR_w, weight];
   
% Initialize variables
H_distance = 0.0;   

%% BEGINNING OF FLIGHT
% Call geometry subroutine
% dummy = 1.0;
[weight,S_w,c_w,AR_w,taper_w,sweep_w,geom] = geometry(dummy);
W_by_S = weight / S_w;

% Call stability subroutine
[StaticPitchMoment,StaticStability] = stability(geom, b_wing,c_w,taper_w);

% Call velocity subroutine
[V_optimal, V_wind, L_by_D, CM, q, CL_optimal] = velocity(V_guess, method); ... (V_optimal in ft/s)
V_opt=V_optimal*0.592484 ... (kts)
                    %    distance = L_by_D * ( (V_optimal - V_wind) / V_optimal ) 1000;
                    %    H_distance = H_distance;... + distance;
                    %    V_guess = V_optimal;	% set next guess to current optimum

% [V_optimal, V_wind, L_by_D, CM] = velocity(V_guess, method) 

% FLIGHT TIME
[P_R, t_flight, range] = Pow_f(V_optimal,weight);
    
Total_distance = range        % total distance traveled (nmi)
                    % H_distance = (V_optimal - V_wind)*T_max; % Distance Traveled (ft)
                    % H_distance = H_distance / 6076.1155;	% convert distances from ft to nmi

ALTITUDE = 0
[windspeed, P, rho, T, mu, nu, mach, REL, q] = atmosphere(dummy,dummy2,V_optimal); 
V_stall = 0.5924838 * sqrt( (2*W_by_S) / (rho*CL_stall))  % (kts)
[windspeed, P, rho, T, mu, nu, mach, REL_stall, q] = atmosphere(dummy,dummy2,V_stall);
% rho
Re = REL*c_w;
Re_stall = REL_stall*c_w;
% Display results:
disp('Aspect Ratio = ');disp(aspectratio);
disp('Endurance = '); disp([num2str(t_flight) ' hours']);
disp('Instantaneous Power Required = '); disp(P_R);
disp('Re at cruise = '); disp(Re);
disp('Re at Stall = '); disp(Re_stall);

Pitch = StaticPitchMoment + CM*q*S_w*c_w;   % [lbf ft]
disp('Static Pitch ='); disp(StaticPitchMoment);
disp('Dynamic Pitch at optimal velocity ='); disp(Pitch);

% Dynamic moment - create visual aid, a plot to show the pitching moment
% applied by the wing throughout velocity range
n = 1;
hold on, % figure
title('Wings Cm effect on pitching moment'), xlabel('velocity [ft/s])'), ylabel('Pitching Moment lbf ft')

% Dr. Roth is a genius try just calling drag for different velocities and
% bypass velocity optimization altogether 

% for n = V_stall:1:70
% for n = 40:1:120
% %     [geom] = Moment_f1(StaticPitchMoment)                   % Was I high?
% %     [V_optimal, V_wind, L_by_D, CM] = velocity(n, method)   % Generate CM (coefficient of moment of wing)
%     [D_total,Cd,Cdp,Cdi,CL] = drag(n);
% %     Pitch = StaticPitchMoment + CM*q*S_w*c_w;                % [lbf ft]
% %     plot(n,Pitch)
%     
%     plot(n, Cd, 'r*', n, Cdp,'go', n, Cdi,'x')
%     legend('Cd','Cdp','Cdi')
%     n = n + 1;
% end
%pause
%% Plots: 
if flag3 == 1.0
    % Plot the wing area:
    [D_zero,Cd,Cdp,Cdi,CL,V_min] = drag(V_optimal);
    visualize(CL_optimal, V_opt, weight, b_wing_in, rho, V_stall)
end


end	% END OF OUTSIDE LOOP

%% Tests

% Code for testing the atmosphere subroutine 
%		Testing complete - results agree to within 1% of those listed in 
%		the standard atmosphere table in Anderson's "Intro to Flight," 4th Ed.

% ========= Not sure when this test was completed - Bryce 2014-15==========
%   However, This is a useful sanity check for atmospheric data - Bryce 2014-15
% =========================================================================

if A_flag == 1.0
	vel = 100;
    figure
    % ========== to make more efficient initiate plot here and:
%     llama = plot(altitude, rho);
        hold on
	for altitude = 100 : 1000 : 60100
      [windspeed, P, rho, T, mu, nu, mach, Re, q] = atmosphere(A_flag,altitude,vel);
      altitude;
      T;
      rho;
      P;
      plot(rho,altitude,'*'), title('Atmospheric Model Verification')
      xlabel('Air Density [slugs/ft^2]'), ylabel('Altitude [ft]')
      % set xdata and ydata of plot here, then 
%      drawnow cmmd to fill out plot
%       set(llama,'
    end
end

%---------------------------------------------------------------------
% Code for testing the velocity subroutine
if V_flag == 1.0
   altitude = 40000;
   V_guess = 200;
% 	[V_optimal, V_wind, L_by_D, CM] = velocity(altitude,V_guess)
    [V_optimal, V_wind, L_by_D, CM] = velocity(V_guess, method);
   
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
   pause 
   figure(1)
%    figure
   plot(V2,D2)
   title('Drag vs. Airspeed')
   figure(2)
%    figure
   plot(V1,obj)
   title('Objective vs. Airspeed')
end
%-------------------------------------------------------------------

% Code for testing the drag subroutine
if D_flag == 1.0

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

