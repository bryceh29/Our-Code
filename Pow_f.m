function [P_R, t_flight, range, eff_motor] = Pow_f(V_optimal,weight)

% This function calculates Power required, total flight time available, and
% range of flight.  
%
% Inputs:   V_optimal   [ft/s]
%           weight      [lb]
%           
% Outputs:  P_R         [W]
%           t_flight    [hours]
%           range       [nmi]
%           
%
%--------------------------------------------------------------------------


% Battery Characteristics:
 Batt_cap = 5000;   % mAh
 Batt_V = 14.8;      % V    
 eff_motor = 0.9;   % Motor Efficiency


[D_zero,Cd,Cdp,Cdi,CL,V_min] = drag(V_optimal); 

W = weight * 4.44822162;            % convert weight to N
LDr = CL/Cd;                        % Calculate lift to drag ratio
T = W/LDr;                          % Calculate required thrust
V= V_optimal * 0.3048;                  % convert V_optimal to m/s
V_opt_kts = V_optimal * 0.592484;       % convert V_optimal to knots
% P_R = W/(CL/Cd)*V / eff_motor; %+ 2;    % [W] (the 2 is what the payload (camera) uses)
P_R = 3*T^1.44;     % Power(thrust) model for KDE 3510XF-475 motor 

I_req = P_R/Batt_V;                     % current required from battery in [A]
t_flight = Batt_cap / (1000 * I_req);   % flight time in hours
range = t_flight*V_opt_kts;             % [nmi]

return