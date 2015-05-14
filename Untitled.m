% Brian Roth
% ENGR 475 - Mechanics of Flight
% Prandtl's Lifting Line Theory
%   Modeling the Lift and CL Distributions on a Wing

close all
clear all
clc

% Wing Geometry and Flight Characteristics
b = 2.4384;   % wing span (m)
AR = 8;      % aspect ratio
S = b^2/AR;  % wing area (m^2)
l = 0.75;  % taper ratio, lambda
cr = 2*S / ((1+l)*b);   % root chord length (m)
ct = l*cr;  % tip chord length (m)
W = 23.943; % takeoff weight (N)
m = W/9.81;  % mass (kg)
%W = 0.95*W; % weight at beginning of cruise is 95% of max takeoff weight
V = 9.8167;    % cruise velocity (m/s)
h = 5;         % cruise altitude  (m)
rho = 1.225;   % air density at sea level (kg/m^3)
q = 0.5*rho*V^2; % dynamic pressure (N/m^2)

% Define Geometry Variations as a Function of Span
N = 10;     % number of Fourier series terms
theta = linspace(pi/(2*N), pi/2, N);    % (pi/2)*[1/N, 2/N, ..., N/N]
y = 0.5*b*cos(theta);       % transform from theta to y
c = cr - (cr-ct)*(2*y/b);   % chord as a fcn of spanwise location
alpha_g = 3.7*pi/180*ones(1,N);   % geometric angle-of-attack (rad)
alpha_L0 = -2.67*pi/180*ones(1,N);   % zero-lift angle-of-attack (rad)

% Formulate Set of Linear Equations
%   [a]{A} = [r]
for i = 1:N   % loop over spanwise locations
    k = 1;
    for j = 1:2:(2*N-1)   % loop over odd Fourier coefficients
        a(i,k) = ((2*b)/(pi*c(i)) + j/sin(theta(i))) * sin(j*theta(i));
        k = k+1;
    end
end

for i=1:N
    r(i) = alpha_g(i) - alpha_L0(i);
end
A = a\r';    % solve for the Fourier series coefficients

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
e = CL^2 / (pi*AR*CDi);  % span efficiency

% Plot Results
figure(1)
plot(y/(b/2),cl)
xlabel('y / Semi-span')
ylabel('Section Lift Coefficient')
title('Section Lift Coefficient Distribution')

figure(2)
plot(y,Lp)
xlabel('Distance from Center of Aircraft (m)')
ylabel('Lift (N/m)')
title('Lift Distribution')
CL_target = W / (q*S);  % lift coefficient to maintain steady, level flight
v = axis;
xrel = 0.31;         % establish a relative position for text
yrel = 0.5;vspace = (v(4)-v(3))/20; % spacing for 25 lines in the box
px = xrel*(v(2)-v(1)) + v(1);       % x coordinate for text
py = yrel*(v(4)-v(3)) + v(3);       % y coordinate for text
text(px,py-vspace, ...
    ['Lift Coefficient = ' num2str(CL,4)],'backgroundcolor',[1 1 1])
text(px,py-2.25*vspace, ...
    ['C_{L,target} = ' num2str(CL_target,4)],'backgroundcolor',[1 1 1])
text(px,py-3.5*vspace, ...
    ['Induced Drag Coefficient = ' num2str(CDi,4)],'backgroundcolor',[1 1 1])
text(px,py-4.5*vspace, ...
    ['Span Efficiency Factor = ' num2str(e,4)],'backgroundcolor',[1 1 1])