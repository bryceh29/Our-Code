% Velocity subroutine

function [V_optimal, V_wind, L_by_D] = velocity(h,V,design)

% Input variables:
%		h   		Geometric altitude (ft)
%		V           Flight velocity (airspeed) (ft/s)
%       design      Vector of wing design variables
%
% Output variables: 
%		V_optimal   Optimal airspeed (ft/s)
%		V_wind		Windspeed at current altitude (ft/s)
%       L_by_D      Lift-to-drag ratio at V_optimal
%
% Description of Method:
%   This subroutine finds the airspeed that maximized range.
%   Assumptions:
%       (1) V_airspeed ~ constant, so L ~ constant
%   Objective Fcn: 
%       f = [V_airspeed/(V_airspeed-V_headwind)] * (D/L)
%       -> f = [V_airspeed/(V_airspeed-V_headwind)] * D    (since L~const)
%       
%--------------------------------------------------------------------

step = 1.0;	% initialization
flag1 = 0;	% initialize warning flag
flag2 = 0;	% initialize warning flag

ctr = 1.0;	% initialize counter
while (step >= 0.0025)
   
   dx = step / 100;	%set the finite differencing step size
   V_plus = V + dx;
   V_minus = V - dx;
   
   % Call drag subroutine
   [D_zero] = drag(h,V,design);
   [D_plus] = drag(h,V_plus,design);
   [D_minus] = drag(h,V_minus,design);
   
   % Call atmosphere subroutine to find windspeed at altitude
   [V_wind] = atmosphere(h,V);

   if (V_wind == 0)
        fp = (D_plus - D_minus) / (2*dx);	% central difference approx. to the 1st derivative
        fpp = (D_plus - 2*D_zero + D_minus) / (dx)^2; % central difference approx to the 2nd deriv.
   
        V_old = V;
        V = V_old - (fp / fpp);	% Newton's method
        step_old = step;
        step_new = abs(V - V_old);
   else
        f_zero = ( V / (V - V_wind) ) * D_zero;
        f_plus = ( V_plus / (V_plus - V_wind) ) * D_plus;
        f_minus = ( V_minus / (V_minus - V_wind) ) * D_minus;
        fp = (f_plus - f_minus) / (2*dx);	% central difference approx. to the 1st derivative
        fpp = (f_plus - 2*f_zero + f_minus) / (dx)^2; % central difference approx to the 2nd deriv.
      
        V_old = V;
        V = V_old - (fp / fpp);	% Newton's method
        step_old = step;
        step_new = abs(V - V_old);      
   end
   
   % Add checks to improve the robustness of Newton's method
   if step_new < step_old
        step = step_new;
   else
        step = step_old;
        V = 0.5*(V + V_old);
   end

   if fpp == 0
        'Second derivative equals zero.'
        'See velocity subroutine.'
        return
   elseif fpp < 0
        V = V_old + 5;	% guess another velocity
        step = step_old;
        flag1 = flag1 + 1;
        if flag1 >= 3
            V = 100;	% try a new starting point
            step_new = 1.0;
        elseif flag1 > 5
            h
            'Second derivative is negative.'
            return
        end
   elseif (100*fpp < fp)
        flag2 = flag2 + 1;
        if flag2 > 5
            h
            '2nd deriv. is much smaller than 1st deriv.'
            return
        end
   end
   
   ctr = ctr + 1;
   if ctr >= 25
        V_optimal = 0.5*(V_old + V);
        'Velocity subroutine exceeded max iterations'
        h	% output the altitude
        break, break
   end
   
end

% Calculate lift & drag
[weight,S_w,c_w] = geometry(design);
[windspeed, P, rho, T, mu, nu, mach, Re, q] = atmosphere(h,V);

%h
%V
%RE_wing = Re * c_w

V_optimal = V;
L_by_D = weight / D_zero;

return