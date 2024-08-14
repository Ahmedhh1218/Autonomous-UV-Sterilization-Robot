% Actuator Sizing
% Constants

Rw      =   0.0325; % Wheel Radius
Miu_r   =   0.04;   % Coefficient of rolling resistance
M       =   1.0335; % Total Mass per wheel
g       =   9.81;   % Acceleration due to gravity
Cd      =   0.27;   % Coefficient of drag (Assumed)
Ro_air  =   1.2;    % Density of air
Af      =   0.031;  % Frontal area
V       =   0.2;    % Robot Velocity
Jwheel  =   3.87e-5;% Wheel Inertia
Jshaft  =   2.59e-7;% Shaft Inertia
t       =   0.25;   % step time
eta     =   0.9;    % Efficiency
Miu_f   =   0.3;    % Coefficient of friction

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Formulas

Teffort_max =   Miu_f * M * g;
Rr          =   Miu_r * M * g;
Ra          =   (1/2) * Cd * Af * Ro_air * (V^2);
Tr          =   (Rr + Ra) * Rw;
Jload_eff   =   (Jwheel / eta) + ( Jshaft / eta);
Jlinear_eff =   M * (Rw^2);
Jmotor      =   Jload_eff;
Jtotal      =   Jmotor + Jload_eff + Jlinear_eff;
alpha_1     =   ((V/Rw) / t);
Tm          =   Tr + (Jtotal * alpha_1);
Teffort     =   Tm / Rw;

Tm_pos_acc  =   Tr + (Jtotal * alpha_1);
Tm_zero_acc =   Tr;
Tm_neg_acc  =   Tr + (Jtotal * -alpha_1);
Tm_rms      =   sqrt(((Tm_pos_acc^2)*t) + ((Tm_zero_acc^2)*2*t) + ((Tm_neg_acc^2)*t));
