

function [StaticPitchMoment,df] = Moment_Handle(x)
% Static Stability objective function 
%
% Dev. by Bryce Hill 2014/15 
% HUAV senior Project
%
% Function: Enables optimization functions changes in geometry information,
% 'x' in this case, to update GEOM file
% 
% Inputs:
%       x       (Objective variable of fmincon in 'Stability' routine
% 
% Calls:
%       Moment_f1(x)    Updates GEOM file with new values
%       Moment_f2(geom) Calculates static moment i.e. due to weight of
%                       components
% 
% Outputs:
%       Static Pitching Moment
%       df

% Call geometry to update geometry information using optimization variable
% 'x'
[weight,S_w,c_w,AR_w,taper_w,sweep_w,geom] = geometry(x);

[StaticPitchMoment] = Moment_f1(geom);
StaticPitchMoment = (StaticPitchMoment-1/12)^2;      % Manually entering effect of CM during analysis, can be removed
df = 0;