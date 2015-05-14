
function [StaticPitchMoment] = Moment_f1(geom)  % Calculate moments about c.g. 
% Static stability moment calculation
% 
% Author:   Bryce Hill
% 
% Dev. by Bryce Hill 2014/15 
% HUAV senior Project
% 
% Calculate static moment about c.g. assumed to be at the 1/4 chord [lb ft]
% 
% To-do:
%       Up to Date!!
   
% ===== 1st iteration   
%         StaticPitchMoment = geom(2,1).weight*geom(2,1).Lx + ...     % Battery
%                         geom(3,1).weight*geom(3,1).Lx + ...     % Tail
%                         geom(3,2).weight*geom(3,2).Lx + ...     % Wing
%                         geom(3,3).weight*geom(3,3).Lx + ...     % Carbon tail
%                         geom(3,4).weight*geom(3,4).Lx + ...     % Carbon Boom
%                         geom(3,5).weight*geom(3,5).Lx + ...     % Fuselage
%                         geom(3,6).weight*geom(3,6).Lx + ...     % Carbon Rod connectors
%                         geom(4,1).weight*geom(4,1).Lx + ...     % Forward Vert Motor
%                         geom(4,3).weight*geom(4,3).Lx + ...     % Rear Vert Motors
%                         geom(4,2).weight*geom(4,2).Lx;          % Push Motor

% ===== 2nd Iteration
%                       E = struct2table(geom(1,1:6));
%                       Pa = struct2table(geom(2,1:6));
%                       S = struct2table(geom(3,1:6));
%                       Pr = struct2table(geom(4,1:6));
% 
% ===== Current - way better to use a table huh!                        
Table = struct2table(geom(1:24));       % 24 = geom rows*columns
StaticPitchMoment = sum(Table.weight.*-Table.Lx);
