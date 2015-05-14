% Optimization subroutine

function [V_optimal] = newton(V)

% Input variables:
%		V           Guess for optimal flight velocity (airspeed) (ft/s)
%
% Output variables: 
%		V_optimal   Optimal airspeed (ft/s)
%
% Description of Method:
%   This subroutine uses a Newton-based optimizer to find the airspeed 
%       that maximizes range.
%   Assumptions:
%       (1) V_airspeed ~ constant, so L ~ constant
%   Objective Fcn:  (to minimize) 
%       f = [V_airspeed/(V_airspeed-V_headwind)] * (D/L)
%       -> f = [V_airspeed/(V_airspeed-V_headwind)] * D    (since L~const)
%       
%--------------------------------------------------------------------

step = 1.0;	% initialization
boundary = 0; % tracks stall speed violation
flag1 = 0;	% initialize warning flag
flag2 = 0;	% initialize warning flag
iter = 50;  % set max iterations

ctr = 1.0;	% initialize counter
while (step >= 0.10) %(step >= 0.0025)
    
   dx = step / 100;	%set the finite differencing step size
   V_plus = V + dx;
   V_minus = V - dx;
   
   % Call drag subroutine
   [D_zero,Cd,Cdp,Cdi,CL,V_min] = drag(V);
   [D_plus] = drag(V_plus);
   [D_minus] = drag(V_minus);
   
   % Call atmosphere subroutine to find windspeed at altitude
   [V_wind, P, rho, T, mu, nu, mach, Re, q, a] = atmosphere(V);
   
   if (V_wind == 0)
        fp = (D_plus - D_minus) / (2*dx);	% central difference approx. to the 1st derivative
        fpp = (D_plus - 2*D_zero + D_minus) / (dx)^2; % central difference approx to the 2nd deriv.
   
        V_old = V;
        V = V_old - (fp / fpp);	% Newton's method
        if (V < V_min)
            if (boundary == 1)  % boundary is almost certainly the feasible minima
                V_optimal = V_min;
                return
            end
            boundary = 1;
            V = V_min;
        end
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
        if (V < V_min)
            if (boundary == 1)  % boundary is almost certainly the feasible minima
                V_optimal = V_min;
                return
            end
            boundary = 1;
            V = V_min;
        end
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
        'See newton subroutine.'
        return
   elseif fpp < 0
        V = V_old + 5;	% guess another velocity
        step = step_old;
        flag1 = flag1 + 1;
        if flag1 >= 3
            V = 100;	% try a new starting point
            step_new = 1.0;
        elseif flag1 > 5
            % h
            'Second derivative is negative.'
            return
        end
   elseif (100*fpp < fp)
        flag2 = flag2 + 1;
        if flag2 > 5
            % h
            '2nd deriv. is much smaller than 1st deriv.'
            return
        end
   end
   
   ctr = ctr + 1;
   if ctr >= iter
        V_optimal = 0.5*(V_old + V);
        'Newton subroutine exceeded max iterations'
        % h	% output the altitude
        break, break
   end
end

V_optimal = V;

return