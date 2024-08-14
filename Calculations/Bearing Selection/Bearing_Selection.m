% Bearing Selection
% Variables
Mt = 0.044  ;       % Mt    ==> Wheel Mass
g  = 9.81   ;       % g     ==> Acceleration due to gravity
L1 = 22.5   ;       % L1    ==> Distance between wheel and bearing 1
L2 = 21     ;       % L2    ==> Distance between bearing 1 and bearing 2
L  = L1 + L2;       % L     ==> Distance between bearing 1 and wheel

% Measuring reaction on bearing in the vertical plane
syms Bx1;           % Bx1   ==> Reaction force on bearing 1
syms Bx2;           % Bx2   ==> Reaction force on bearing 2

% Taking moment about bearing 1
Mb1 = Mt * g * L - Bx2 * L2;
Bx2 = vpasolve(Mb1 == 0, Bx2);

% Taking moment about bearing 2
Mb2 = Mt * g * L1 - Bx1 * L2;
Bx1 = abs(vpasolve(Mb2 == 0, Bx1));

% No force in the horizontal plane
By1 = 0;            % By1   ==> Reaction force on bearing 1 in the horizontal plane
By2 = 0;            % By2   ==> Reaction force on bearing 2 in the horizontal plane
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% So the reaction force on each bearing will be the vertical plane forces
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% From strandard
X  = 1;
Y  = 0;
Co = 5.2;          % Co    ==> Maximum static load the bearing can handle
V  = 1;             % V     ==> Constant depend on whether the shaft is fixed or rotating

% Calculate bearing rev per million life B
Lh = 10000;         % Lh    ==> Number of bearing working hours
N  = 100;           % N     ==> the shaft rotational speed in rpm
B = Lh * N * 60 / 10 ^ 6;

% Bearing 1 calculation
Fr1 = sqrt(Bx1^2 + By1^2);  % Radial force on bearing 1
Fa1 = 0;                    % axial force on bearing 1
Fe1 = X * V * Fr1 + Y * Fa1;
C1calc = Fe1 * (B^(1/3))    % calculate static load on bearing 1
if (C1calc < Co)
    disp('Bearing 1 Valid!')
else
    disp('Invalid Bearing 1 Selection!')
end
% C1calc < Co ==> bearing is suitable

% Bearing 2 calculations
Fr2 = sqrt(Bx2^2 + By2^2);  % radial force on bearing 2
Fa2 = 0;                    % axial force on bearing 2
Fe2 = X * V * Fr2 + Y * Fa2;
C2calc = Fe2 * (B^(1/3))    % calculate static load on bearing 2
if (C1calc < Co)
    disp('Bearing 2 Valid!')
else
    disp('Invalid Bearing 2 Selection!')
end
% C2calc < Co ==> bearing is suitable