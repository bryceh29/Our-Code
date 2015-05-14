% Objective subroutine

function [f,g] = Objective(V)

% Input variables:
%		V           Guess for optimal flight velocity (airspeed) (ft/s)
%
% Output variables: 
%		f           drag
%		g           d(drag)/dV
%
% Description of Method:
%   This subroutine returns the drag & its 1st derivative w.r.t. velocity      
%--------------------------------------------------------------------------

dx = 0.001;	%set the finite differencing step size (Newton: dx=0.0001)
V_plus = V + dx;
V_minus = V - dx;

% Call drag subroutine
f = drag(V);
[D_plus] = drag(V_plus);
[D_minus] = drag(V_minus);

g = (D_plus - D_minus) / (2*dx);	% central difference approx. to the 1st derivative

return