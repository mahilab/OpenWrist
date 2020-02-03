% Evan Pezent | evanpezent.com | epezent@rice.edu
% 02/04/2017

function joint = ParseSolidworksMassProps(filename)
% =========================================================================
% Parses Solidworks mass properties dialog when the output of the "Copy to
% Clipboard" button is pasted into a text file. Decimals is how many
% numbers a reported after the decimal point for each property. Tested with
% Solidworks 2016.

% IMPORTANT NOTE:
% The Solidworks Mass Properties dialog gives positive inertia tesor:
%  Ixx  Ixy  Ixz
%  Iyx  Iyy  Iyz
%  Izx  Izy  Izz
% Not the "negative" inertia tensor per 6.16 of Craig (3e):
%  Ixx -Ixy -Ixz
% -Iyx  Iyy -Iyz
% -Izx -Izy  Izz

% http://help.solidworks.com/2016/English/SolidWorks/sldworks/HIDD_MASSPROPERTY_TEXT_DLG.htm
% https://forum.solidworks.com/message/445422
% http://mathworld.wolfram.com/MomentofInertia.html
% =========================================================================

mp = fileread(filename); % read in mass properties file
mp = mp(6:end); % remove first instance of the word 'Mass'

    function val = findVal(varname)
        var_ind = strfind(mp,varname);
        index = var_ind+length(varname)+3;
        try
            val = mp(index:index+8);
        catch
            val = mp(index:index+7);
        end
    end

% Joint Mass [kg]
joint.m = eval(findVal('Mass'));

% Joint Center of Mass: [X,Y,Z]' [m]
joint.Xc = eval(findVal('X'));
joint.Yc = eval(findVal('Y'));
joint.Zc = eval(findVal('Z'));
joint.Pc = [joint.Xc;joint.Yc;joint.Zc];

% Moments of inertia take at COM & aligned with output coordinate system
% (L) [kg*m^2]
joint.Icxx = eval(findVal('Lxx'));
joint.Icxy = eval(findVal('Lxy'));
joint.Icxz = eval(findVal('Lxz'));
joint.Icyx = eval(findVal('Lyx'));
joint.Icyy = eval(findVal('Lyy'));
joint.Icyz = eval(findVal('Lyz'));
joint.Iczx = eval(findVal('Lzx'));
joint.Iczy = eval(findVal('Lzy'));
joint.Iczz = eval(findVal('Lzz'));
joint.Ic = [+joint.Icxx -joint.Icxy -joint.Icxz;
            -joint.Icyx +joint.Icyy -joint.Icyz;
            -joint.Iczx -joint.Iczy +joint.Iczz];

% Moments of inertia taken at the output coordinate system 
% (I) [kg*m^2]
% I = Ic + m*(Pc'*Pc*eye(3)-Pc*Pc') (kholsa1985)
joint.Ixx = eval(findVal('Ixx'));
joint.Ixy = eval(findVal('Ixy'));
joint.Ixz = eval(findVal('Ixz'));
joint.Iyx = eval(findVal('Iyx'));
joint.Iyy = eval(findVal('Iyy'));
joint.Iyz = eval(findVal('Iyz'));
joint.Izx = eval(findVal('Izx'));
joint.Izy = eval(findVal('Izy'));
joint.Izz = eval(findVal('Izz'));
joint.I = [+joint.Ixx -joint.Ixy -joint.Ixz;
           -joint.Iyx +joint.Iyy -joint.Iyz;
           -joint.Izx -joint.Izy +joint.Izz];

end