% Vehicle Dynamic Stability Subroutine
% Dev. by Bryce Hill 2014/15 
% HUAV senior Project

% Inputs:
%       geom - vehicle geometry information specified in "geometry"
%              subroutine
%       q (dynamic pressure) []
% Outputs:
%       DynamicPitchMoment
%       DynamicStability
% To-do:

% Function:
% - Performs moment by weight defined in "geometry" subroutine about 1/4 
%   chord
% - Plot top, side views for visual understanding of design

% function [DUM] = stability(AR_w S_w c_wing b_wing V_v V_h w_batt)
function [DynamicPitchMoment,DynamicStability] = DynamicStability(geom, q)
geom = geom;            % Cannot rename structs apparently? oh yeah no I had a clear all in there

%% Dynamic Stability
% Run Optimization routing 'fmincon' SQP
%     Objective   | Dynamic Pitch Moment
%     
%     Constraints | Minimize weight & Moment
%     
%     Variables   | Lx battery, fuselage & booms
%                 | W fuselage & booms

% Working
X0 = [geom(2,1).Lx          % Battery moment arm
      geom(4,1).Lx];        % Forward motors moment arm
  
LB = [0  .75];      % Lower bounds on variables
UB = [9.5/12 1.5];  % Upper bounds on variables
A = [0 0;           % Coefficients of linear constraints: I don't have any
     0 0];     
B = [0; 0];         % Right side vector for linear constraints

OPTIONS = optimoptions('fmincon','Algorithm','sqp');
[x] = fmincon('Moment_f1',X0,A,B,[],[],LB,UB,[],OPTIONS);

[DynamicPitchMoment] = Moment_f2(geom);

DynamicStability(1).category = 'Moment arm';
DynamicStability(1).Lx = x(1);
DynamicStability(2).category = 'Moment arm';
DynamicStability(2).Lx = x(2);
DynamicStability(3).category = 'Static Pitch';
DynamicStability(3).Lx = DynamicPitchMoment;

disp('Battery Moment Arm [in]')
x(1)
disp('Motor Moment Arm [in]')
x(2)

%% temporary for development, remove when completed
b_w = b_wing;

od = .591/12;              % (in)
lt = 3;                    % total boom length(in)
myl = lt/1.75;             % moment boom arm (y hat)
mxl = geom(3,4).Lx;        % moment boom arm (x hat)
% Tail
c_h = 6/12;                % Horizontal tail chord
% Fuselage
Fw = 4/12;              % Width fuselage
Fl = 1;                 % Length fuselage
Fm = 2/12;              % Offset from quarter cord of wing and cg fuselage (+y)

