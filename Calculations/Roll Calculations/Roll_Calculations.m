% Roll Calculations
% Variables
Mass    = 4.13419   ;
L       = 215.02    ;   % L     ==> Wheel base
L1      = 107.35    ;   % L1    ==> Rear wheel to Cg
L2      = 107.67    ;   % L2    ==> Front wheel to Cg
g       = 9.81      ;   % g     ==> Acceleration due to gravity
hcg     = 59.78     ;   % hcg   ==> Height of Cg
W       = 350       ;   % W     ==> Wheel track

% Stability in the longitudinal direction
% Static weight
Nr = Mass * g * L2 / L; % Nr    ==> Reaction on rear wheel
Nf = Mass * g * L1 / L; % Nf    ==> Reaction on front wheel

% Weight transfer in the longitudinal direction
syms a  ;               % a     ==> Max acceleration without flipping
syms Nfd;               % Nfd   ==> Static Weight + Weight transfer
syms Nrd;               % Nrd   ==> Static Weight - Weight transfer

% Taking moment about front wheel
Nrd = Nr + Mass * a * hcg / L;

% Taking moment about front wheel
Nfd = Nf - Mass * a * hcg / L;

% Longitudinal roll over condition
a = vpasolve(Nfd == 0 , a);

% Stability in the lateral direction
% Static weight
Nout = Mass * g / 2;    % Nout  ==> Reaction on outer wheel
Nin  = Mass * g / 2;    % Nin   ==> Reaction on inner wheel

% Weight transfer in lateral direction
syms v  ;               % v     ==> Max velocity without turning over a certain corner
syms No ;               % No    ==> Static Weight + Weight transfer
syms Ni ;               % Ni    ==> Static Weight - Weight transfer
R = 0.3 ;               % R     ==> Corner radius

% Taking moment about inner wheel
No  = Nout + Mass * v^2 * hcg / W / R;

% Taking moment about front wheel
Ni  = Nin - Mass * v^2 * hcg / W / R;

% Lateral roll over condition
v = max(vpasolve(Ni == 0 , v));