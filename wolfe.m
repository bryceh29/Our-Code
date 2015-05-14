% Author:   Brian D. Roth
% Date:     February 9, 2005 
% File:     wolfe
%
% Purpose: A univariate minimization algorithm
%           (Also referred to as Wolfe's method?)
%
% Inputs:
%   P   - Search direction
%   x   - Current point
%   f   - Function value at the current point
%   g   - Gradient (vector) at the current point
%
% Outputs:
%   x_star  - New value of x (a vector)
%
% Local Variables:
%   alpha
%   flag - flag=1 -> minimum between min(x) & max(x)
%          flag=2 -> minimum extrapolated
%          flag=3 -> no minimum (x describes a concave surface)
%          flag=4 -> stationary point identified
%          flag=5 -> minimum is already contained in [alpha_temp]
%   eta - Determines the accuracy with which alpha approximates a
%           stationary point of F along P. Provides a means of controlling
%           the balance of effort to be expended in computing alpha
%           (0 <= eta < 1, where eta=0 corresponds to an exact line search)
%   mu  - Guarantees a suitable reduction in F
%           (0 < mu <= 1/2, where mu = 0 implies no required reduction)
%           (Note: Typically, select mu <= eta)
%   xt  - Trial value of x. If all line search criteria are met, then xt
%           becomes x_star
%
%   temp, t,
%   i,j,k   - Integer indices
%   m       - Vector of indices
%
%   References:
%       Gill, Murray, & Wright, "Practical Optimization," Elsevier Academic
%       Press, London, England, 1986.
%
%       Murray, Walter, MS&E 315 Course Notes, pgs. 21-31, January, 2005.
%
%-------------------------------------------------------------------------

function [x_star] = wolfe(P,x,xb,f,g)

% General Parameters:
eta = 0.20; %0.2;
mu = 0.05;  %0.05;
alpha = 1.0;
n = length(x);  % number of variables

% If bounds on variables are exceeded, adjust alpha
xt = x + alpha * P;
a_new = ones(1,length(x));
for j = 1:n
    if ( xt(j) > xb(j,2) )  % check upper bounds
        a_new(j) = ( xb(j,2)- x(j) ) / ( xt(j) - x(j) );
    elseif ( xt(j) < xb(j,1) )  % check lower bounds
        a_new(j) = ( xb(j,1)- x(j) ) / ( xt(j) - x(j) );
    end
end
alpha = min(a_new); % create new alpha

% Parameters used for safeguarded polynomial interpolation
xt = x + alpha * P;
[f_xt, g_xt] = Objective(xt);
alpha_temp = [0 alpha];     % vector of current three best values of x
f_temp = [f f_xt];    % vector of fcn values corresponding to x_temp

% Univariate minimization with safeguarded polynomial updates to step length
i = 1;
while (i <= 20)
    if ( (abs(g_xt' * P) <= (-eta * g' * P))...        % slope criteria
        & (( f - f_xt ) >= ( -mu*alpha * g' * P )) );  % fcn value criteria
            x_star = xt;
            return
    else    % select new alpha
        if i == 1
            alpha = 0.5 * alpha;
            xt = x + alpha .* P;
            [f_xt, g_xt] = Objective(xt);
            alpha_temp = [alpha_temp, alpha];   f_temp = [f_temp, f_xt];
        else    % for all i > 1
            % This is the heart of the algorithm
            if (i > 2) % identify & discard the "worst" value of x
                
                % CASE 1: The function value based on alpha is the best yet
                if (f_xt < min(f_temp))
                    % Case 1.1 - the three pts in alpha_temp do not bound alpha
                    if ( (alpha < min(alpha_temp)) | (alpha > max(alpha_temp)) )
                        [temp,j] = max(f_temp);
                        f_temp(j) = f_xt;   alpha_temp(j) = alpha;
                    % Case 1.2 - alpha is bounded by the 3 pts in alpha_temp
                    %   Keep nearest pts on either side of xt
                    else
                        [t,m] = sort(alpha_temp);
                        for j = 1:3
                            t(j) = t(j) - alpha;
                        end
                        % eliminate t with same sign as t(2)
                        if t(2) < 0     % eliminate t(1)
                            alpha_temp(m(1)) = alpha;  f_temp(m(1)) = f_xt;
                        elseif t(2) > 0     % eliminate t(3)
                            alpha_temp(m(3)) = alpha;  f_temp(m(3)) = f_xt;
                        else
                            'Error #1 - See line search algorithm'
                            return
                        end
                    end
                    
                % CASE 2: The function value based on alpha is > max(f_temp)
                elseif ( f_xt > max(f_temp) )
                    [alpha_temp, m] = sort(alpha_temp);
                    f_temp = f_temp(m);
                    % Case 2.1 - alpha provides a new bounds on f_min
                    if ( (alpha < min(alpha_temp)) & (f_temp(1) == min(f_temp)) )
                        alpha_temp(3) = alpha;  f_temp(3) = f_xt;
                    elseif ( (alpha > max(alpha_temp)) & (f_temp(3) == min(f_temp)) )
                        alpha_temp(1) = alpha;  f_temp(1) = f_xt;
                    else    % new pt is essentially useless
                        'Line Search Algorithm Failed - Error #2'
                        return
                    end
                    
                % CASE 3:  f_min < f_xt < f_max  (could be an equality too)
                else
                    [alpha_temp, m] = sort(alpha_temp);
                    f_temp = f_temp(m); 
                    % Case 3.1 - f_new provides a previously unknown bounds on f_min 
                    %   Keep new bound & drop the pt with the largest f
                    if ( (alpha > alpha_temp(3)) & (f_temp(3) == min(f_temp)) )
                        alpha_temp(1) = alpha;  f_temp(1) = f_xt;
                    elseif ( (alpha < alpha_temp(1)) & (f_temp(1) == min(f_temp)) )
                        alpha_temp(3) = alpha;  f_temp(3) = f_xt;
                    % Case 3.2 - alpha is less desirable than existing pts
                    elseif ( (alpha < min(alpha_temp)) | (alpha > max(alpha_temp)) )
                        'Line Search Algorithm Failed - Error #3'
                        return
                    % Case 3.3 - alpha tightens the bound on alpha_best
                    elseif ( f_temp(1) == min(f_temp) )
                        f_temp(3) = f_xt;   alpha_temp(3) = alpha;
                    elseif ( f_temp(3) == min(f_temp) )
                         f_temp(1) = f_xt;  alpha_temp(1) = alpha;
                    elseif ( (alpha > alpha_temp(2)) & (alpha < alpha_temp(3)) )
                        if (f_xt < f_temp(3))
                            f_temp(3) = f_xt;   alpha_temp(3) = alpha;
                        else
                            'Error'
                            return
                        end
                    elseif ( (alpha < alpha_temp(2)) & (alpha > alpha_temp(1)) )
                        if (f_xt < f_temp(1))
                            f_temp(1) = f_xt;   alpha_temp(1) = alpha;
                        else
                            'Error'
                            return
                        end
                    % Case 3.4 - two values of x have the same fcn value
                    %   This is highly improbably, unless these pts
                    %   correspond to a stationary pt.
                    else
                        temp1 = 1;  temp2 = 1;
                        for j = 1:3
                            temp1 = temp1 * (f_xt - f_temp(j));
                            temp2 = temp2 * (alpha - alpha_temp(j));
                        end
                        if ( (temp1 == 0) & (temp2 ~= 0) )
                            'Stationary Point Found'
                            x_star = x + alpha * P;
                            return
                        else
                        % Case 3.5 - an unanticipated case has occurred
                            'Line Search Algorith Failed - Error #4'
                            return
                        end
                    end
                end     
            end
            % Use quadratic polynomial to find best alpha, given current set of three alpha
            [alpha, flag] = Interp(alpha_temp, f_temp);
                        
            if (flag == 4)  % the interpolation fcn has identified a stationary pt
                'Stationary Point Found'
                x_star = x + alpha * P;
                return
            elseif (flag == 5)  % minimum is already contained in [alpha_temp]
                %'Line Search Terminated Early'
                x_star = x + alpha * P;
                return                
            end            
            xt = x + alpha * P;
            [f_xt, g_xt] = Objective(xt);
            
            % If bounds on variables are exceeded, adjust xt
            for j = 1:n
                if ( xt(j) > xb(j,2) )  % check upper bounds
                    xt(j) = xb(j,2);
                    flag == 6;
                elseif ( xt(j) < xb(j,1) )  % check lower bounds
                    xt(j) = xb(j,2);
                    flag == 6;
                end
            end
            % if one (or more) dimensions of the search vector are bounds-limited
            if (flag == 6)   
                [f_xt, g_xt] = Objective(xt);
                if ( f_xt < min(f_temp) ) % if the fcn value is the best yet
                    x_star = xt;
                    return
                else
                    'Need to refine line search w/ bounds - #1'
                    return
                end
            end % end of bounds-checking
        end
    end
    i = i+1;
end

%test(P,x,Prob,alpha_hist)

% return if fcn value criteria is satisfied, even without gradient criteria
'Line Search - Gradient criteria not satisfied'
if ( f - f_xt ) >= ( -mu*alpha * g' * P )
    x_star = xt;
    'Line Search - Satisfied based only on fcn value criteria'
    return
end

% If safeguarded polynomial fitting fails to find a suitable point,
%   then we'll resort to a "backtracking" algorithm to update alpha
'Warning - Line Search based on safeguarded polynomial fitting has failed'
'Beginning line search based on backtracking'
i = 1;
while (i <= 20)
    xt = x + alpha .* P;
    %   If bounds on variables are exceeded, adjust xt
    for j = 1:n
        if ( xt(j) > xb(j,2) )  % check upper bounds
            xt(j) = xb(j,2);
        elseif ( xt(j) < xb(j,1) )  % check lower bounds
            xt(j) = xb(j,1);
        end
    end
    [f_xt, g_xt] = Objective(xt);
    
    if abs(g_xt' * P) <= (-eta * g' * P);   % slope criteria
        if ( f - f_xt ) >= ( -mu*alpha * g' * P );  % fcn value criteria
            x_star = xt;
            return
        else    % select new alpha (can be based on safeguarded poly. fitting)
            alpha = 0.5 * alpha;    % just to get the algorithm up & running
        end
    else    % select new alpha (can be based on safeguarded poly. fitting)
        alpha = 0.5 * alpha;    % just to get the algorithm up & running
    end
    i = i+1;
end
x_star = xt;    % Set x_star equal to last 
'Line search algorithm did not converge.'
return
%--------------------------------------------------------------------------

% REVIEW OF PERTINENT ALGORITHMS

% Safeguarded Polynomial Fitting Algorithm
% Sample 3 points:
%   (1) xt1 = x + alpha*P
%   (2) xt2 = x + 1/2 * alpha * P
%   (3) if f(xt1) < f(xt2)
%           xt3 = x + 1.5 * alpha * P
%       else
%           xt3 = x + 0.75 * alpha * P
%       end

% Alternative Approaches:
% (1) Goldstein Principle - Gill, Murray & Wright, pg. 100

 
% Univariate Minimization Algorithm:
%   Choose eta, such that 0 <= eta < 1
%   Chooce mu, such that 0 < mu <= 1/2 & mu < eta
%
%   alpha <- alpha_0  (usually, alpha_0 = 1)
%   x_(k+1) = x_k + alpha * p_k
%   if |g(x_(k+1))T * p_k| <= -eta * g(x_k)T * p_k
%       if F(x_k) - F(x_(k+1)) >= -mu*alpha*g(x_k)T*p_k
%           keep x_(k+1)
%           end line search
%       else
%           select new alpha (can be based on safeguarded poly. fitting)
%           repeat the process
%       end
%   else
%       select new alpha (can be based on safeguarded poly. fitting)
%       repeat the process
%   end
