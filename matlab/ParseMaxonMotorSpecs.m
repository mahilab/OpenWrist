% Evan Pezent | evanpezent.com | epezent@rice.edu
% 4/8/2106

function motor = ParseMaxonMotorSpecs(filename)
% =========================================================================
% Parses text file containing motor specificatoins copied from the product
% page on Maxon's website (see expected format below). Tested and working
% 4/8/2016. Subject to break with website changes.
% =========================================================================

data = fileread(filename);

motor.w_nl = findVal('No load speed') * (2*pi*1/60);                           % No Load Speed [rad/sec]       = X[rev/min] * (2pi[rad]/1[rev]*1[min]/60[sec])
motor.i_nl = findVal('No load current') * (1/1000);                            % No Load Current [A]           = {X[mA] * (1[A]/1000[mA])
motor.w_nom = findVal('Nominal speed') * (2*pi*1/60);                          % Nominal Speed [rad/sec]       = X[rev/min] * (2pi[rad]/1[rev]*1[min]/60[sec])
motor.tau_nom = findVal('Nominal torque (max. continuous torque)') * (1/1000); % Nominal Torque [Nm]           = X[mNm] * (1[Nm]/1000[mNm])
motor.i_nom = findVal('Nominal current (max. continuous current)');            % Nominal Current [A]
motor.tau_stall = findVal('Stall torque') * (1/1000);                          % Stall Torque [Nm]             = X[mNm] * (1[Nm]/1000[mNm])
motor.i_stall = findVal('Stall current');                                      % Stall Current [A]
motor.n = findVal('Max. efficiency') * 1/100;                                  % Max Efficiency
motor.t_m = findVal('Mechanical time constant') * (1/1000);                    % Mech. Time Constant [s]       = X[ms] * (1[s]/1000[ms])
motor.V = findVal('Nominal voltage');                                          % Nominal Voltage [V]
motor.R = findVal('Terminal resistance');                                      % Terminal Resistance [Ohms]       
motor.L = findVal('Terminal inductance') * (1/1000);                           % Terminal Inductance [H]       = X[mH] * (1[H]/1000[mH])
motor.J = findVal('Rotor inertia') * (1/1000*(1/100)^2);                       % Rotor Inertia [kg-m^2]        = X[g-cm^2] * (1[kg]/1000[g]*(1[m]/100[cm])^2)
motor.Kt = findVal('Torque constant') * (1/1000);                              % Torque Constant [Nm/A]        = X[mNm/A] * (1[Nm]/1000[nNm])
motor.Kv = findVal('Speed constant') * (2*pi*1/60);                            % Speed Constant [rad/s-V]      = X[rev/min-V] * (2pi[rad]/1[rev]*1[min]/60[sec])
motor.Kb = 1/motor.Kv;                                                         % Back EMF Consant [V-s/rad]    = 1/Kv

    function val = findVal(varname)
        var_ind = strfind(data,varname);
        index0 = var_ind+length(varname);
        temp = data(index0:end);
        for ii = 1:length(temp)
            if temp(ii) ~= ' '
                break
            end
        end            
        index1 = index0 + ii - 1;
        temp = data(index1:end);
        index2 = strfind(temp,' '); index2 = index1 + index2(1) -2;
        val = eval(data(index1:index2));       
    end  

end

% The textfile should be look something like this (without the %'s):
% =========================================================================
% Values at nominal voltage
% Nominal voltage 48 V
% No load speed   8490 rpm
% No load current 78.6 mA
% Nominal speed   7760 rpm
% Nominal torque (max. continuous torque) 89.7 mNm
% Nominal current (max. continuous current)   1.74 A
% Stall torque    1050 mNm
% Stall current   19.6 A
% Max. efficiency 88 %
% Characteristics
% Terminal resistance 2.45 Ω
% Terminal inductance 0.513 mH
% Torque constant 53.8 mNm/A
% Speed constant  178 rpm/V
% Speed / torque gradient 8.09 rpm/mNm
% Mechanical time constant    2.94 ms
% Rotor inertia   34.7 gcm²
% Thermal data
% Thermal resistance housing-ambient  6 K/W
% Thermal resistance winding-housing  1.7 K/W
% Thermal time constant winding   16.9 s
% Thermal time constant motor 593 s
% Ambient temperature -30...+100 °C
% Max. winding temperature    +125 °C
% Mechanical data
% Bearing type    ball bearings
% Max. speed  12000 rpm
% Axial play  0.05 - 0.15 mm
% Radial play 0.025 mm
% Max. axial load (dynamic)   5.6 N
% Max. force for press fits (static)  110 N
% (static, shaft supported)   1200 N
% Max. radial load    28 N, 5 mm from flange
% Other specifications
% Number of pole pairs    1
% Number of commutator segments   13
% Number of autoclave cycles  0
% Product
% Weight  260 g
% =========================================================================
