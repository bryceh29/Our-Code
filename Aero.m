%% Adding this to figure out the Git process



function [e] = Aero(V)
% Michael Kudla - Coefficient for loop addition
% edited by Bryce Hill - 14/15
% ENGR 351 - LNA
% Fourier Series Application
% Using a Fourier Series Approximation to Model Lift Distribution
% function [CL, CDi] = 

global DESIGN dummy dummy2
design = DESIGN;

% V = 20.58;    % cruise velocity of 20 kts (m/s)****************************


% Call atmosphere subroutine
[windspeed, P, rho, T, mu, nu, mach, REL, q] = atmosphere(dummy, dummy2, V);
%	Note: REL is the Reynolds number per unit length

% Call geometry
[weight,S_w,c_w,AR_w,taper_w,sweep_w] = geometry(dummy);
% [weight,S_w,c_w,AR_w,taper_w,sweep_w,S_wl,AR_wl] = geometry(1.0);

% Wing Geometry and Flight Characteristics
b = design(1);   % wing span (ft)
c = design(2);
S = S_w;  % wing area (ft^2)
AR = AR_w; % aspect ratio
l = taper_w;  % taper ratio, lambda

cr = 2*S / ((1+l)*b);   % root chord length (m)
ct = l*cr;  % tip chord length (m)
W = 0.95*weight; % weight at beginning of cruise is 95% of max takeoff weight(lb)

h = 200;  % cruise altitude of 200 ft 
 % air density at 39,000 ft (slug/ft^3) is rho
q = 0.5*rho*V^2;% dynamic pressure (N/m^2)*******************************

% Define Geometry Variations as a Function of Span
N = 10;     % number of Fourier series terms
theta = linspace(pi/(2*N), pi/2, N);    % (pi/2)*[1/N, 2/N, ..., N/N]
y = 0.5*b*cos(theta);       % transform from theta to y
c = cr - (cr-ct)*(2*y/b);   % chord as a fcn of spanwise location
alpha_g = 3.612*pi/180*ones(1,N);   % geometric angle-of-attack (rad)
alpha_L0 = -4.0*pi/180*ones(1,N);   % zero-lift angle-of-attack (rad)

% Formulate Set of Linear Equations
%   [a]{A} = [r]
%
%  [Update this section with code to computer Fourier series coefficients]

for i=1:10
    for j=1:10
        a(i,j)=(2*b/(pi()*c(i))+(2*j-1)/sin(theta(i)))*sin((2*j-1)*theta(i));
    end
    B(i)=alpha_g(i)-alpha_L0(i);
end
A=a\B';
% For an elliptic wing   (replace with above section, when completed)
% A = zeros(1,N);
% A(1) = 0.023063;    % extra credit for explaining this value

% Solve for L and Cl distributions
for i = 1:N   % loop over spanwise locations
    gamma(i) = 0;
    for j = 1:N   % loop over Fourier terms
        gamma(i) = gamma(i) + 2*b*V*A(j)*sin((2*j-1)*theta(i));
    end
    Lp(i) = rho*V*gamma(i);
end
cl = Lp ./ (q*c);  % section lift coefficient

% Solve for CL, CDi, and e
CL = pi*AR*A(1);   % wing lift coefficient
sum = 0;
for i = 2:N
    sum = sum + (2*i-1)*(A(i)/A(1))^2;
end
CDi = pi*AR*A(1)^2 * (1+sum);   % induced drag
e = CL^2 / (pi*AR*CDi); % span efficiency

% Plot Results
% figure(1)
% plot(y/(b/2),cl)
% xlabel('y / Semi-span')
% ylabel('Section Lift Coefficient')
% title('Section Lift Coefficient Distribution')
% 
% figure(2)
% plot(y,Lp)
% xlabel('Distance from Center of Aircraft (m)')
% ylabel('Lift (N/m)')
% title('Lift Distribution')
% CL_target = W / (q*S);  % lift coefficient to maintain steady, level flight
% v = axis;
% xrel = 0.31;         % establish a relative position for text
% yrel = 0.5;vspace = (v(4)-v(3))/20; % spacing for 25 lines in the box
% px = xrel*(v(2)-v(1)) + v(1);       % x coordinate for text
% py = yrel*(v(4)-v(3)) + v(3);       % y coordinate for text
% text(px,py-vspace, ...
%     ['Lift Coefficient = ' num2str(CL,4)],'backgroundcolor',[1 1 1])
% text(px,py-2.25*vspace, ...
%     ['C_{L,target} = ' num2str(CL_target,4)],'backgroundcolor',[1 1 1])
% text(px,py-3.5*vspace, ...
%     ['Induced Drag Coefficient = ' num2str(CDi,4)],'backgroundcolor',[1 1 1])
% text(px,py-4.5*vspace, ...
%     ['Span Efficiency Factor = ' num2str(e,4)],'backgroundcolor',[1 1 1])
return