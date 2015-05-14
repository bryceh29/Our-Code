
function [StaticPitchMoment,StaticStability] = stability(geom,b_wing,c_w,taper_w)
% Vehicle Static Stability Subroutine
% Dev. by Bryce Hill 2014/15 
% HUAV senior Project
%
% Inputs:
%       b_w     wing span
%       c_w     wing chord
%       taper_w taper ratio
% 
% Calls:
%       Moment_Handle.m
%           - 
% Outputs:
%       StaticPitchMoment
%       StaticStability
% To-do:
%       Cg location
%       Plot detailing component layout 
%
% Function:
% - Performs moment by weight defined in "geometry" subroutine about 1/4 
%   chord
% - Plot top, side views for visual understanding of design
%
% function [DUM] = stability(AR_w S_w c_wing b_wing V_v V_h w_batt)         

%% Static Stability
% Run Optimization routing 'fmincon' SQP
%     Objective   | Static Pitch Moment
%     
%     Constraints | Minimize weight & Moment
%     
%     Variables   | Lx battery, fuselage & booms
%                 | W fuselage & booms

% Working
X0 = [geom(2,1).Lx          % Battery moment arm
      geom(4,1).Lx];        % Forward motors moment arm
  
% LB = [0  .75];      % Lower bounds on variables
LB = [0  0];
% UB = [9.5/12 1.5];  % Upper bounds on variables
UB = [24/12 15/12];
A = [0 0;           % Coefficients of linear constraints: I don't have any
     0 0];     
B = [0; 0];         % Right side vector for linear constraints

OPTIONS = optimoptions('fmincon','Algorithm','sqp');
[x] = fmincon('Moment_Handle',X0,A,B,[],[],LB,UB,[],OPTIONS);

[StaticPitchMoment] = Moment_f1(geom);

StaticStability(1).category = 'Moment arm';
StaticStability(1).Lx = x(1);
StaticStability(2).category = 'Moment arm';
StaticStability(2).Lx = x(2);
StaticStability(3).category = 'Static Pitch';
StaticStability(3).Lx = StaticPitchMoment;

disp('Battery Moment Arm [in]')
x(1)
disp('Motor Moment Arm [in]')
x(2)


