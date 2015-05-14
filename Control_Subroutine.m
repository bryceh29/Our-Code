
% Control Surface Sizing Subroutine

[Ab, Ac, Eb, Ec, Rb, Rc] = control(AR_w S_w c_wing b_wing)

% Control Subrouting Inputs:
% =========================================================================
% Average Chord
% Surface Area_wing
% Aspect Ratio

% Control Subroutine Outputs:
% =========================================================================
% Ab = Aileron Span     | Ac = Aileron Chord
% Eb = Elevator Span    | Ec = Elevator Chord
% Rudder Span           | Rc = Rudder Chord

% Call Globally Defined input variables
AR = AR_w;              
S_w = Sw;               
c_wing = Cw;
b_wing = bw;

Ab = 
Spanr = Ab/bw;                  % Ratio of area to 

