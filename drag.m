% drag subroutine

function [D_total,Cd,Cdp,Cdi,CL,V_min,D_i,D_p,CM] = drag(V)
global AIRFOIL AIRFOIL_WL
airfoil = AIRFOIL;
airfoil_wl = AIRFOIL_WL;
% Input variables: 
%		V		velocity (airspeed) (ft/s)
%		h		altitude (ft)
%		
% Output variables:
%		cd		drag coefficient
%		cdp     parasite (viscous+pressure) drag coef.
%		cdi     induced drag coef.
%
% Approach:
%		This approach is different from that typically used for sizing commercial aircraft.
%		  In the traditional approach, the skin friction coefficient is estimated from experimental
%		data for fully turbulent flow over flat plates. However, RC aircraft operate on the lower 
%		bounds for this data. In addition, the data is based on a fully turbulent boundary layer. 
%		  In contrast, we anticipate a portion of laminar flow and, at times, laminar separation. 
%		Thus, a better approach is to use windtunnel data for specific airfoils of interest. 
%		This data includes both skin friction & pressure drag due to separation.
%
% Simplifying assumptions:
%		(1) The Mach number is sufficiently low (M < 0.5) so that aerodynamic heating does not 
%				significantly impact the boundary layer (and skin friction coefficient). 
%		(2) A form factor is usually included in skin friction estimates to account for 
%				Mach #, t/c, and sweep. The portion due to thickness is already included in the
%				airfoil data. It is assumed that the Mach # & wing sweep are sufficiently small
%				so that their impact on skin friction can be ignored.
%		(3) A form factor can also be added to account for the impact of fuselage interference
%				on the skin friction of the wing. We choose to ignore this since the fuselage
%				is small and the wing has a relatively high aspect ratio.
%		(4) A form factor IS included to account for drag increments associated with roughness
%				resulting from typical construction procedures. The form factor also accounts for 
%				drag associated with push rods, external camera?, etc.
%		(5) Oswald's efficiency factor includes span efficiency and fuselage interference.
%				It does not include lift dependent pressure drag - this is already included in the
%				airfoil data.
%       (6) XFOIL suggests a small, but non-negligible correlation between 
%               CDP and Mach number. This is strictly a subsonic effect. 
%               XFOIL obviously doesn't capture transonic effects. 
%-----------------------------------------------------------------------------------------

global DESIGN dummy dummy2

CL_max = DESIGN(3); %%%%%% Will want to change

[E] = Aero(V);
FF = 1.4;	% Form factor (see #4 above)

% Call atmosphere subroutine
[windspeed, P, rho, T, mu, nu, mach, REL, q] = atmosphere(dummy,dummy2,V);
%	Note: REL is the Reynolds number per unit length

% Call geometry
[weight,S_w,c_w,AR_w,taper_w,sweep_w] = geometry(dummy);
% [weight,S_w,c_w,AR_w,taper_w,sweep_w,S_wl,AR_wl] = geometry(1.0);
b = sqrt(AR_w * S_w);

%% BOOMS

%l_boom = 3.5 - c_w





%% Vertical motors




%% WING - DRAG CALCULATION

Re_w = REL * c_w;
CL = weight / (q*S_w);		% lift coef.
% CL = 1;
    %   disp(CL);
V_min = V * sqrt(CL / CL_max);  % stall velocity (ft/s) NOT SURE WHERE THIS COMES FROM
    %	Note: This assumes a small glide angle such that lift ~ weight

% Parasite drag
[Cdp_w, CM] = drag_polar(Re_w,CL); %I think this does viscid

% Induced drag coef.
CL_ht = 0.2;
S_ht = 1;
CL_w = CL + CL_ht * (S_ht / S_w);   % Add lift needed to compensate for tail downforce
Cdi_w = CL_w^2 / (pi * AR_w * E); %This is from Stanford, Drag, inviscid-induced

%% % FUSELAGE - DRAG CALCULATION
            
            l_fuse = 2; %ft
            w_fuse = 4/12; %ft
            h_fuse = 2/12; %ft
            Acs_fuse_max = (h_fuse)*(w_fuse);
            D_eff_fuse = (4*Acs_fuse_max/pi)^.5;  %This comes from stanford form factor notes
            
           Re_f = REL * l_fuse;
            % frictional coef - this approx. assumes turbulent, incompressible flow
            cf_fuse = 0.455 / (log10(Re_f))^2.58;
 
%             ff_ffuse = [1.41 1.29 1.219 1.172 1.139 1.115 1.098 1.084 1.073]
%             Fineness = [4 5 6 7 8 9 10 11 12]
            
            ff_fuse = 1.139;
            S_wet_fuse = 2*h_fuse*l_fuse+2*l_fuse*w_fuse; %Comes from wetted area calcs stanford notes... revisit on final design!
            S_wet_fuse = S_wet_fuse + 2 * (pi*0.55/12)*2; %Add area of 2 booms (0.55" diameter x 2 ft length)
            
            Cd_p_fuse = ff_fuse*cf_fuse*S_wet_fuse/S_w;  % nondimensionalize by wing area 
            
%             Note: Impact of angle of incidence on fuselage drag is ignored

%% TAILS
            % HORIZONTAL TAIL - DRAG CALCULATION
            AR_ht = 4;
            S_ht = 1;
            CL_ht = 0.2;
            c_ht = sqrt(S_ht / AR_ht);
            Re_ht = REL * c_ht;
            
            % % Parasite drag
            Cdp_ht = drag_polar(Re_ht,CL_ht);
            Cdp_ht = Cdp_ht * (S_ht / S_w);	% nondimensionalize by wing area
            
            % Induced drag coef.
            Cdi_ht = (CL_ht)^2 / (pi * AR_ht * E);
            Cdi_ht = Cdi_ht * (S_ht / S_w);	% nondimensionalize by wing area

            % VERTICAL TAIL - DRAG CALCULATION
            S_vt = .7;
            AR_vt = 1.2;
            CL_vt = 0.05;		% no rudder deflection for trimmed condition
            c_vt = sqrt(S_vt / AR_vt);
            Re_vt = REL * c_vt;
            
            % Parasite drag
            Cdp_vt = drag_polar(Re_vt,CL_vt);
            Cdp_vt = Cdp_vt * (S_vt / S_w);	% nondimensionalize by wing area
            
            % Induced drag coef.
            Cdi_vt = (CL_vt)^2 / (pi * AR_vt * E);
            Cdi_vt = Cdi_vt * (S_vt / S_w);	% nondimensionalize by wing area


%% WINGLETS - DRAG CALCULATION

% CL_wl = 0.2;		
% c_wl = sqrt(S_wl / AR_wl);
% Re_wl = REL * c_wl;
% 
% % Parasite drag
% Cdp_wl = drag_polar(airfoil_wl,Re_wl,CL_wl);
% Cdp_wl = Cdp_wl * (S_wl / S_w);	% nondimensionalize by wing area
% 
% % Induced drag coef.
% Cdi_wl = (CL_wl)^2 / (pi * AR_wl * E);
% Cdi_wl = Cdi_wl * (S_wl / S_w);	% nondimensionalize by wing area


%% TOTAL DRAG
Cdp = FF * (Cdp_w + Cdp_vt + Cdp_ht + Cd_p_fuse);
% Cdp = FF * (Cdp_w + Cdp_wl);
Cdp = Cdp + 0.0042*mach^4;  % account for effect of Mach number
Cdi = Cdi_w + Cdi_vt + Cdi_ht;
% Cdi = Cdi_w + Cdi_wl;
Cd = Cdp + Cdi;
D_total = q * S_w * Cd;
D_i = q * S_w * Cdi;
D_p = q * S_w * Cdp;

% Write out results
% 'write out data'
% V = sqrt(2*q/rho)
% CL
% Re_w
% D_total
% weight
% Cdp_w
% Cd_p_fuse
% Cdp_vt
% Cdp_ht
% Cd
% Cdi

            % % TOTAL DRAG (Ignoring Winglets)
            % Cdp = FF * (Cdp_w);
            % Cdp = Cdp + 0.0042*mach^4;  % account for effect of Mach number
            % Cdi = Cdi_w;
            % Cd = Cdp + Cdi;
            % D_total = q * S_w * Cd;

