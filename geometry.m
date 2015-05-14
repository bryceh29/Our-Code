% geometry subroutine
%%%%Add engine propulsion to a new subroutine
function [weight,S_w,c_w,AR_w,taper_w,sweep_w,geom] = geometry(dummy)
% Updated by Bryce Hill 2014/15
% 
% Inputs:
%       dummy:  if dummy == 0 (specified in main)
%                   not running optimization routine, values user defined
%               else 
%                   geom values edited by optimization routing input 
% Outputs: 
%       Aircraft weight & geometry
%
% Calls:
%       none
% 
% Called by:
%       main
%       drag
%       velocity
%       Moment_f1
%
% Weights Estimate (in pounds unless specified otherwise)
%		Fixed weight: GPS, camera+beacon, control servos
%		Variable weight: foam core, fiberglass skin
%		Discrete weight: tail boom(s), tail (rudder & elevator)
%		Optional weight: pressure probe
%
% Tail Sizing method
%       Tail volume: V = l S / cw Sw
%           l ~ (1/2) * 36" = 1.5 ft
%           S ~ area of horizontal or vertical tail
%           cw = wing chord
%           Sw = wing area
%       Initial estimates:  (from Alex Haas)
%           V_h = 0.40
%           V_v = 0.07
% 
% Units:
%       geom.weight [lbs]
%       geom.Lx     [in]
%--------------------------------------------------------------------------

global DESIGN
design = DESIGN;
x = dummy;
%% GEOMETRY DEFINITION
% -------------------
% Wing geometry
b_w = design(1);
c_w = design(2);
tc_w = 0.129;       % wing t/c for MH81
S_w = b_w * c_w;
AR_w = b_w / c_w;
taper_w = 1.0;      % taper ratio (implicit in the defn of b, c, & S)
sweep_w = 0.0;      % sweep

% Fuselage geometry
fuse_l = 3.0;           % length (ft)
S_fuse = (6/12) * (8/12) * fuse_l;
S_fuse = 0.80 * S_fuse;  % reduce area to acct for taper of nose & tail

% Vertical tail
V_v = 0.07;             % tail volume
L = 1.5;                % tail moment arm (ft)
S_vt = V_v * c_w * S_w / L;
AR_vt = 2.0;            % just a guess
b_vt = sqrt(AR_vt * S_vt);
c_vt = S_vt / b_vt;
tc_vt = tc_w;           % t/c

% Horizontal tail
V_h = 0.4;              % tail volume
L = 1.5;                % tail moment arm (ft)
S_ht = V_h * c_w * S_w / L;
b_ht = 1.0;             % span limited to 1 ft
c_ht = S_ht / b_ht;
AR_ht = b_ht / c_ht;
tc_ht = tc_w;           % t/c
CL_tail = -0.2;         % CL est. for trimmed flight


%% WEIGHT ESTIMATION (lb)
% -----------------
rho_foam = 1.0;                 % foam density (lb/ft^3)
rho_balsa= 12.0;                % balsa density (lb/ft^3)
rho_glass= 3.1;                 % midweight fiberglass (oz/yd^2)
rho_glass= rho_glass/16/9;      % fiberglass (lb/ft^2)
rho_glass= 2.0 * rho_glass;     % glass + epoxy (lb/ft^2)
rho_plywd= 50.0;                % plywood density (lb/ft^3)
rho_carb = 90.0;                % carbon fiber density (lb/ft^3)
% linear_weight = 0.01702;        % linear weight carbon       -- not used
rho_abs  = 64.972;              % abs 3D plastic (lb/ft^3)
% rho_corPlastic = 14.86;         % Corrugated plastic (lb/ft^3)
rho_corPlastic = .1551;         % Corrugated plastic (lb/ft^3)
% -----------------

w_misc = 7 / 16;                % mountings, glue, etc.
 
%% ELECTRONICS (1,:)
w_cptr = .7760/16;              % Piccolo Avionics board (22 grams)
    geom(1,1).category = 'Electronics';
    geom(1,1).name = 'Computer';
    geom(1,1).weight = w_cptr;
    geom(1,1).Lx = 0/12;
w_gps = .4585/16;               % Piccolo GPS (13 grams)
    geom(1,2).category = 'Electronics';
    geom(1,2).name = 'GPS';
    geom(1,2).weight = w_gps;
    geom(1,2).Lx = 0/12;
w_receiver = 1.0229/16;         % Piccolo receiver
    geom(1,3).category = 'Electronics';
    geom(1,3).name = 'Receiver';
    geom(1,3).weight = w_receiver;
    geom(1,3).Lx = 0/12;
w_electronics = w_cptr + w_receiver + w_gps;
    geom(1,4).category = 'Electronics';
    geom(1,4).name = 'Total';
    geom(1,4).weight = w_electronics;
    geom(1,4).Lx = 0/12;
    
    geom(1,5).category = 'Electronics';
    geom(1,5).name = 'Blank1';
    geom(1,5).weight = 0;
    geom(1,5).Lx = 0/12;
    
    geom(1,6).category = 'Electronics';
    geom(1,6).name = 'Blank2';
    geom(1,6).weight = 0;
    geom(1,6).Lx = 0/12;
    

%% PAYLOAD (2,:)
w_batt = 20.1 / 16;                  % battery (5000 50C)
    geom(2,1).category = 'Payload';
    geom(2,1).name = 'Battery';
    geom(2,1).weight = w_batt;
% -------- Optimization option --------------------------------------------
    if (dummy == 0)                 % Means optimizer in 'Stability' routine
        geom(2,1).Lx = 9/12;        % is not running, so use default value
    else
        geom(2,1).Lx = x(1);        % Where 'x' is optimization variable
    end
% -------------------------------------------------------------------------    
w_serv = 4 * (3*0.035274) / 16;      % servos (pololu PN 2141)
    geom(2,2).category = 'Payload';
    geom(2,2).name = 'Servos';
    geom(2,2).weight = w_serv;
    geom(2,2).Lx = -19.5/12;
w_payload = w_serv + w_electronics + w_batt;
    geom(2,3).category = 'Payload';
    geom(2,3).name = 'Total';
    geom(2,3).weight = w_payload;
    geom(2,3).Lx = 0;
    
    geom(2,4).category = 'Payload';
    geom(2,4).name = 'Blank1';
    geom(2,4).weight = 0;
    geom(2,4).Lx = 0;
    
    geom(2,5).category = 'Payload';
    geom(2,5).name = 'Blank2';
    geom(2,5).weight = 0;
    geom(2,5).Lx = 0;
    
    geom(2,6).category = 'Payload';
    geom(2,6).name = 'Blank3';
    geom(2,6).weight = 0;
    geom(2,6).Lx = 0;
    
%% STRUCTURE (3,:)
    % Wing & Tail
V_wing = .1204;                     % ft^3
% V_tail = V_v+V_h;                 % [ft^3]
V_Tail = 1.479;  %[ft^2, area not volume]                    % Estimate
% w_tail = V_tail * rho_foam;
w_tail = V_Tail*rho_corPlastic;
    geom(3,1).category = 'Structure';
    geom(3,1).name = 'Tail';
    geom(3,1).Lx = -23/12; 
    geom(3,1).weight = w_tail;
w_wing = V_wing * rho_foam;
    geom(3,2).category = 'Structure';
    geom(3,2).name = 'Wing';
    geom(3,2).Lx = 0; 
    geom(3,2).weight = w_wing;
w_foam = w_tail+w_wing;             % foam wing & tail weight

    % Carbon booms multiplied 
ro = (.5512/12)*.5;                      % Outer diameter 15 mm carbon rods 
ri = (.4724/12)*.5;                      % Inner Diameter 12 mm carbon rods
rom = (.3937/12)*.5;                        % 10 [mm] diameter
rim = (.3149/12)*.5;                        % 8 [mm] diameter
ros = (.3149/12)*.5;                        % 8 [mm] diameter
ris = (.2362/12)*.5;                        % 6 [mm] diameter

L_carbon_t1 = 24/12;                            % Elevator Section [ft]
L_carbon_t2 = 8.75/12;                          % Rudder section [ft]
L_carbon_x = (39*2)/12;
L_carbon_y_big = 2;                             % Length of CENTRAL wing support section [ft]
L_carbon_y_small = 4;                           % Length of TIP wing support section [ft]
L_carbon_yTotal = L_carbon_y_big + L_carbon_y_small;     % total length of carbon spars [ft]
    % Define boom volume by moment arm
    
V_carbon_t = pi*(rom^2-rim^2)*L_carbon_t1 + pi*(ros^2-ris^2)*L_carbon_t2;    
V_carbon_x = pi*(ro^2-ri^2)*L_carbon_x;
V_carbon_y = pi*(ro^2-ri^2)*L_carbon_y_big + pi*(rom^2-rim^2)*L_carbon_y_small;

w_carbon_t = V_carbon_t * rho_carb;
    geom(3,3).category = 'Structure';
    geom(3,3).name = 'Carbon Tail';
    geom(3,3).Lx = -30/12;
    geom(3,3).weight = w_carbon_t;
    
w_carbon_x = V_carbon_x*rho_carb;
    geom(3,4).category = 'Structure';
    geom(3,4).name = 'x-axis Carbon Boom';
% -------- Optimization option --------------------------------------------
    if (dummy == 0)                 % Means optimizer in 'Stability' routine
        geom(3,4).Lx = -7.5/12;     % is not running, so use default value
    else
        geom(3,4).Lx = (-24/12-x(2))/2+x(2);  % Where 'x' is optimization variable
        L_carbon_x = 24/12+x(2);
        V_carbon_x = pi*(ro^2-ri^2)*2*L_carbon_x;
        geom(3,4).weight = V_carbon_x*rho_carb;    % Boom weight
    end
% -------------------------------------------------------------------------
    geom(3,4).weight = w_carbon_x;


    geom(3,5).category = 'Structure';
    geom(3,5).name = 'Fuselage';
    geom(3,5).Lx = (c_w + 4 + geom(2,1).Lx)/(2*12); % 4 is extra distance fuselage must extend to accomodate battery   
        % Calculate weight of fuselage
        r_fuse = 1.75/12;                       % Radius of fuselage [in]
        h = 2/12;                               % Nose & Tail taper height
        V_fuselage = pi*r_fuse^2*geom(3,5).Lx+1/3*pi*r_fuse^2*h;    % Volume fuselage (past value .0347)
        w_fuselage = V_fuselage*rho_foam;       % Weight fuselage
        %w_fuselage = .1595;                    % 1st iteration weight
    geom(3,5).weight = w_fuselage;
    
% Attachments
V_attachments = 0;                  % Carbon rod connectors
w_attachments = V_attachments * rho_abs;
w_attachments = .06;
    geom(3,6).category = 'Structure';
    geom(3,6).name = 'Attachments';
    geom(3,6).Lx = geom(3,3).Lx;                % rear two only DEFINED BY CARBON ROD LENGTH
    geom(3,6).weight = w_attachments;


%% PROPULSION (4,:)
w_prop_vert  = 1.06/16;            % 14x5.5MR (1.06oz) 
w_prop_push  = 1.06;               % Not sure yet
w_motor_vert = 2.54/16;            % 3Dr AC2836-358
    geom(4,1).category = 'Propulsion';
    geom(4,1).name = 'Vertical Motors Forward';
% -------- Optimization option --------------------------------------------    
    if (dummy == 0)                 % Means optimizer in 'Stability' routine
        geom(4,1).Lx = 14.5/12;     % is not running, so use default value
    else
        geom(4,1).Lx = x(2);        % Where 'x' is optimization variable
    end
% -------------------------------------------------------------------------
    geom(4,1).weight = 2*w_prop_vert;
    
    geom(4,3).category = 'Propulsion';
    geom(4,3).name = 'Vertical Motors Rear';
%     geom(4,3).Lx = -13/12;
    geom(4,3).Lx = -7/12;
    geom(4,3).weight = 2*w_prop_vert;
w_motor_push = 7.05/16;            % Estimate based on vert motors
    geom(4,2).category = 'Propulsion';  
    geom(4,2).name = 'Pusher Motor';
    geom(4,2).Lx = -4/12;
    geom(4,2).weight = w_prop_push;
    
    geom(4,4).category = 'Propulsion';  
    geom(4,4).name = 'Blank1';
    geom(4,4).Lx = 0/12;
    geom(4,4).weight = 0;
    
    geom(4,5).category = 'Propulsion';  
    geom(4,5).name = 'Blank2';
    geom(4,5).Lx = 0/12;
    geom(4,5).weight = 0;
    
    geom(4,6).category = 'Propulsion';  
    geom(4,6).name = 'Blank3';
    geom(4,6).Lx = 0/12;
    geom(4,6).weight = 0;

%% TOTAL WEIGHT
% Use table conversion to access every 'weight' element and sum
Table = struct2table(geom(1:24));       % 24 = geom rows*columns
weight = sum(Table.weight)+w_misc;

% OVERRIDE WEIGHT TO USE MEASURED WEIGHT OF TOTAL AIRCRAFT:

