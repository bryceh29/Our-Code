% Author:   Brian D. Roth
% Date:     February 27, 2005 
% File:     Interpolation
%
% Purpose:
%   Create a quadratic polynomial & return its minimizer
%
% Inputs:
%   a   - A vector of three pts at which the function value is known
%   f   - A vector of function values at [a]
%
% Outputs:
%   x_star  - Minimizer of the quadratic
%   f_star  - Function value at the minimizer of the quadratic
%   flag    - flag=1 -> minimum between min(a) & max(a)
%             flag=2 -> minimum extrapolated
%             flag=3 -> no minimum (x describes a concave surface)
%             flag=4 -> stationary point identified
%             flag=5 -> minimum is already contained in [a]
%
%--------------------------------------------------------------------------

function [a_star, flag] = Interp(a,f)

% CHECK INCOMING MATRICES FOR POTENTIAL TROUBLE
temp = ( a(1)-a(2) ) * ( a(1)-a(3) ) * ( a(2)-a(3) );
if (temp == 0)
    'Error - Must provide three independent data pts to function Interp'
    return
end
if ( (max(f) - min(f)) < 1E-12 ) % stationary pt identified
    a_star = median(a);
    flag = 4;
    return
end
if ( (abs(f(1)-f(2)) < 1E-12) | (abs(f(1)-f(3)) < 1E-12) | (abs(f(2)-f(3)) < 1E-12))
    if ( abs(f(1)-f(2)) < 1E-12 )
        a_star = ( a(1) + a(2) ) / 2;
    elseif ( abs(f(1)-f(3)) < 1E-12 )
        a_star = ( a(1) + a(3) ) / 2;
    else
        a_star = ( a(2) + a(3) ) / 2;
    end
    flag = 5;
    return
end


% SOLVE FOR THE COEFFICIENTS THAT DEFINE A QUADRATIC FIT
xm = [ a(1)^2 a(1) 1;
       a(2)^2 a(2) 1;
       a(3)^2 a(3) 1];
A = inv(xm) * f';

% CASE 1 - The quadratic defines a concave surface
if A(1) < 0
    flag = 3;
    [temp,j] = max(f);
    if ( a(j) == max(a) )   % if f is increasing as alpha increases
        a_star = min(a) - 0.5*(max(a)-min(a));  % Decrease alpha
    elseif ( a(j) == min(a) ) % if f is decreasing as alpha increases
        a_star = 1.25 * max(a);    % Increase alpha
    else    % [a] bound a local maxima - highly unlikely, but conceivable
        [temp,j] = max(a);
        if ( f(j) == min(f) )
            a_star = 1.25 * max(a);    % Increase alpha
        else
            a_star = min(a) - 0.5*(max(a)-min(a));  % Decrease alpha
        end
    end
    
% CASE 2 - The quadratic defines a convex surface
else
    % Test for special cases
    if ( A(1) == 0 )    % can only occur at a stationary pt
        flag = 4;
        a_star = median(a);
        return
    elseif ( A(2) == 0 )
        if ( (a(1)==0) | (a(2)==0) | (a(3)==0) )
            flag = 5;
            a_star = 0;
        else
            flag = 1;
            a_star = 0;
        end
        return
    end
    % Solve for the minimum of the quadratic approx
    a_min = -A(2) / (2*A(1));
    
    % Case 2.1:  'a_min' is bounded by 'a'
    if ( (a_min > min(a)) & (a_min < max(a)) ) 
        flag = 1;
        a_star = a_min;
        return
    % Case 2.2 - 'a_min' is equal to one of the elements in 'a'
    elseif ( (a_min==a(1)) | (a_min==a(2)) | (a_min==a(3)) )
        flag = 5;
        a_star = a_min;
        return
    %Case 2.3 - a_min is not bounded by min(a) and max(a)
    %   Place bounds (or "safe guards") on step length
    else
        flag = 2;
        if a_min < min(a)
            temp = min(a) - 10.25 * ( max(a)-min(a) );
            a_star = max(a_min, temp);
        elseif a_min > max(a)
            temp = max(a) + 10.25 * ( max(a)-min(a) );
            a_star = min(a_min,temp);
        else
            'Error in Safeguarded Polynomial Fitting Algorithm'
        end
    end
end

return