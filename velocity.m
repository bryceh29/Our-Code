% Velocity subroutine
%%%Once we have modeled engine performance we need to include those specs.
function [V_optimal, V_wind, L_by_D, CM, q, CL_optimal] = velocity(V, method)
global dummy dummy2
% Input variables:
%		h   		Geometric altitude (ft)
%		V           Guess for optimal flight velocity (airspeed) (ft/s)
%
% Output variables: 
%		V_optimal   Optimal airspeed (ft/s)
%		V_wind		Windspeed at current altitude (ft/s)
%       L_by_D      Lift-to-drag ratio at V_optimal
%
% Description of Method:
%   This subroutine finds the airspeed that maximizes range.
%   Assumptions:
%       (1) V_airspeed ~ constant, so L ~ constant
%   Objective Fcn: 
%       f = [V_airspeed/(V_airspeed-V_headwind)] * (D/L)
%       -> f = [V_airspeed/(V_airspeed-V_headwind)] * D    (since L~const)
%       
%--------------------------------------------------------------------------

% Define optimization method
%   1 - Newton-based optimizer
%   2 - Safe-guarded polynomial fitting
method = method;        % Defined in main
    
if (method == 1)
    [V_optimal] = newton(V);
    [D_zero,Cd,Cdp,Cdi,CL,V_min] = drag(V_optimal);

elseif (method == 2)
    % Call Objective subroutine
    P = 1.0;             % search direction
    Vb = [10.0, 100.0]; % bounds on V
    [f,g] = Objective(V);
    [V_optimal] = wolfe(P,V,Vb,f,g);
    [D_zero,Cd,Cdp,Cdi,CL,V_min,D_i,D_p,CM] = drag(V_optimal);
  
else
    'Please specify an optimizer in the velocity subroutine.'
    return
end
    
% Calculate lift & drag
[weight,S_w,c_w] = geometry(dummy);
[V_wind, P, rho, T, mu, nu, mach, Re, q, a] = atmosphere(dummy, dummy2, V);
[Drag] = drag(V_optimal);
Drag
L_by_D = weight / Drag;
disp('CL at V_optimal = '); disp(CL);
CL_optimal = CL;
% disp('RE = '); disp(Re);

return