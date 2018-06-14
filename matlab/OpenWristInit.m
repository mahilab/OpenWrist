% Evan Pezent | evanpezent.com | epezent@rice.edu
% 02/04/2017

function openWrist = OpenWristInit()
% =========================================================================
% Initializes constant parameters for the OpenWrist.
% =========================================================================

%% Mass and Inertia (Solidworks Version: rev5_20161009)
% JOINT 1: PS (FX/FE/RU Supressed, Values Relative to {1})
openWrist.PS = ParseSolidworksMassProps('sw_mp_ps.txt');

% JOINT 2: FE (FX/PS/RU Supressed, Values Relative to {2})
openWrist.FE = ParseSolidworksMassProps('sw_mp_fe.txt');

% JOINT 3: RU (FX/PS/FE Supressed, Values Relative to {3})
openWrist.RU = ParseSolidworksMassProps('sw_mp_ru.txt');

%% Damping Coefficients [Nm*s/rad]
openWrist.PS.B = 0.0252;  
openWrist.FE.B = 0.0019;  
openWrist.RU.B = 0.0029; 

%% Kinetic Friction  [Nm]
openWrist.PS.fk = 0.1891; 
openWrist.FE.fk = 0.0541;
openWrist.RU.fk = 0.1339; 

%% Static Friction [Nm]
openWrist.PS.fs = 0.2250;
openWrist.FE.fs = 0.0720;
openWrist.RU.fs = 0.1180;

%% Motor Properties
openWrist.PS.motor = ParseMaxonMotorSpecs('maxon_148877.txt');
openWrist.FE.motor = ParseMaxonMotorSpecs('maxon_148877.txt');
openWrist.RU.motor = ParseMaxonMotorSpecs('maxon_310009.txt');

%% Transmission Ratios [in/in]
openWrist.PS.eta = 8.75 / 0.468;
openWrist.FE.eta = 9.00 / 0.468;
openWrist.RU.eta = 6.00 / 0.234;

%% Other Constants
openWrist.g = 9.80665; % gravity constant [m/s^2]

end








