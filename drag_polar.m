% drag_polar subroutine
% z = Drag, drag polar refers to CL vs Cd plot

function [CDP CM] = drag_polar(RE,CL)
global AIRFOIL AIRFOIL_WL
airfoil = AIRFOIL;
airfoil_wl = AIRFOIL_WL;
% Input variables: 
%		RE		Reynolds number
%		CL		lift coefficient
%		
% Output variables:
%		CDP		parasite drag coefficient of the lifting surface
%
% Notes:
%       The SD7027 airfoil was chosen because it is popular for RC sailplanes.
%       The ARES airfoil was chosen because it was developed for a low RE, high Mach mission.
%--------------------------------------------------------------------------------------------

%   0 - NACA 5310 airfoil
%   1 - ARES airfoil with quadratic drag polar representation
% 	2 - SD7037 with single quadratic drag polar representation
%	3 - SD7037 with split quadratic drag polar representation
%   4 - ARES airfoil
%   5 - MH81 flying wing airfoil
%   6 - PSU 94-097 winglet airfoil
%   7 - E 205 RC glider airfoil 



if airfoil == 0 % NACA 5310 airfoil with drag polar via interpolation
%% AIRFOIL 0
    x = [50000, 75000, 100000, 150000, 200000, 250000, 300000];     % Reynolds number
    y = [0.00 0.10 0.20 0.30 0.40 0.50 0.60 0.70 0.75 0.780 0.80 0.90 0.95 1.00 1.2];  % Lift Coefficient
    z(1,:) = [.03943 0.03354 0.02874 0.02497 0.02178 0.01968 0.01929 .01994 .02137 .02280 .02271 .03141 .035 .04 .06];
    z(2,:) = [.040265 .03205 .02467 .01936 .01537 .01184 .00981 .00940 .00975 .01007 .01038 .01208 .01326 .01480 .18];
    z(3,:) = [.04110 .03298 .02246 .01597 .01173 .00869 .00686 .00670 .00697 .00723 .00733 .00871 .00952 .01035 .012];
    z(4,:) = [.04181 .03232 .02426 .01143 .00789 .00576 .00463 .00467 .00492 .00510 .00527 .00622 .00683 .00747 .01];
    z(5,:) = [.04393 .03064 .01385 .00868 .00590 .00439 .00374 .00385 .00409 .00427 .00439 .00520 .00569 .00621 .0082];
    z(6,:) = [.04285 .02870 .01977 .00685 .00467 .00361 .00326 .00342 .00363 .00379 .00392 .00461 .00503 .00552  .0075];
    z(7,:) = [.03897 .02719 .00949 .00593 .00402 .00310 .00296 .00313 .00333 .00349 .00359 .00423 .00460 .00505  .0065];

    % CAUTION: z(2,1), z(1,13), and z(1,14) are not obtained from XFOIL.  XFOIL
    % could not converge to find these values, so they are interpolated from
    % the data on either side of them using the higher and lower re.
    


    % Pseudo Code
    % (1) Bound CL with y[]
    if (CL < 0.0)
        '!!! WARNING: CL < 0.0'
        return
    elseif (CL > 1.2)
        '!!! WARNING: CL > 1.2'
        return
    end
    i = 1;
    % y(1);
    while CL >= y(i)
        i = i+1;
    end
    CLL = i-1;   % lower bound on CL index
    CLU = i;

    % (2) Bound RE with x[]
    if (RE < 50000) % Approximate lower Reynolds numbers using data for RE=50k
        RE = 50000;
    elseif (RE >= 300000) % Approx. higher RE using data for RE = 300k
        RE = 299995;
    end
    % disp('RE = '); disp(RE);
    i2 = 1;
    while RE >= x(i2)
        i2 = i2+1;
    end
    REL = i2-1;   % lower bound on RE
    REU = i2;

    % (3) Interpolate between CLs to find two CDPs
    CDP_L = z(REL, CLL) + (CL-y(CLL))/(y(CLU)-y(CLL)) * (z(REL, CLU) - z(REL, CLL));    % CDP at lower RE
    CDP_U = z(REU, CLL) + (CL-y(CLL))/(y(CLU)-y(CLL)) * (z(REU, CLU) - z(REU, CLL));    % CDP at higher RE

    % (4) Interpolate between CDPs to find CDP
    CDP = CDP_L + (RE-x(REL))/(x(REU)-x(REL)) * (CDP_U - CDP_L);
    if (CL > 1.2)
        CDP
        pause
    end


    elseif airfoil == 4 % ARES airfoil with drag polar via interpolation
%% AIRFOIL 4
    x = [50000, 75000, 100000, 150000, 200000, 250000, 300000];     % Reynolds number
    y = [0.00 0.10 0.20 0.30 0.40 0.50 0.60 0.70 0.75 0.780 0.80 0.90 0.950 1.00];  % Lift coefficient
    z(1,:) = [0.01777 0.02048 0.02402 0.02648 0.02874 0.03062 0.03099 0.02778 0.02474 0.02292 0.02551 0.03479 0.04730 0.05980];
    z(2,:) = [0.01575 0.01756 0.01872 0.01968 0.02034 0.02059 0.02039 0.01827 0.01732 0.01675 0.02274 0.03126 0.03674 0.04974];
    z(3,:) = [0.01402 0.01476 0.01525 0.01551 0.01545 0.01533 0.01515 0.01418 0.01427 0.01433 0.02051 0.02807 0.03509 0.04397];
    z(4,:) = [0.01130 0.01132 0.01124 0.01097 0.01069 0.01062 0.01085 0.01117 0.01164 0.01534 0.01780 0.02514 0.02939 0.03786];
    z(5,:) = [0.01254 0.00941 0.00913 0.00880 0.00856 0.00874 0.00914 0.00986 0.01067 0.01483 0.01593 0.02142 0.02577 0.03012];
    z(6,:) = [0.01164 0.00797 0.00788 0.00757 0.00746 0.00774 0.00827 0.00910 0.01052 0.01394 0.01508 0.01941 0.02271 0.02831];
    z(7,:) = [0.01082 0.00697 0.00706 0.00677 0.00678 0.00711 0.00771 0.00861 0.01114 0.01284 0.01398 0.01794 0.02136 0.02477];

    % Pseudo Code
    % (1) Bound CL with y[]
    if (CL < 0.0)
        '!!! WARNING: CL < 0.0'
        return
    elseif (CL > 1.0)
        '!!! WARNING: CL > 1.0'
        return
    end
    i = 1;
    % y(1);
    while CL >= y(i)
        i = i+1;
    end
    CLL = i-1;   % lower bound on CL
    CLU = i;

    % (2) Bound RE with x[]
    if (RE < 50000) % Approximate lower Reynolds numbers using data for RE=50k
        RE = 50000;
    elseif (RE >= 300000) % Approx. higher RE using data for RE = 300k
        RE = 299995;
    end
    i2 = 1;
    while RE >= x(i2)
        i2 = i2+1;
    end
    REL = i2-1;   % lower bound on RE
    REU = i2;

    % (3) Interpolate between CLs to find two CDPs
    CDP_L = z(REL, CLL) + (CL-y(CLL))/(y(CLU)-y(CLL)) * (z(REL, CLU) - z(REL, CLL));   % CDP at lower RE 
    CDP_U = z(REU, CLL) + (CL-y(CLL))/(y(CLU)-y(CLL)) * (z(REU, CLU) - z(REU, CLL));    % CDP at higher RE

    % (4) Interpolate between CDPs to find CDP
    CDP = CDP_L + (RE-x(REL))/(x(REU)-x(REL)) * (CDP_U - CDP_L);
    if (CL > 1.2)
        CDP
        pause
    end

    elseif airfoil == 1 % ARES with quadratic drap polar representation
%% AIRFOIL 1
        A = -1.64E-6*(RE/1000)^2 + 8.64E-4*(RE/1000)- 8.58E-2;
        B = 1.74E-6*(RE/1000)^2 - 8.57E-4*(RE/1000)+ 8.13E-2;
        C = 1.03E-7*(RE/1000)^2 - 6.06E-5*(RE/1000)+ 1.92E-2;
        CDP = A*CL^2 + B*CL + C;  %parasite drag coef.

    elseif airfoil == 2	% SD7037 with single quadratic drag polar representation
%% AIRFOIL 2
        A = 1.8E-7*(RE/1000)^2 - 6.7E-5*(RE/1000)+ 0.032;
        B = 7.8E-7*(RE/1000)^2 - 0.00025*(RE/1000)+ 0.0026;
        C = 6.7E-7*(RE/1000)^2 - 0.00025*(RE/1000)+ 0.036;
        CDP = A*CL^2 + B*CL + C;  %parasite drag coef.

    elseif airfoil == 3  % SD7037 with split quadratic drag polar representation
%% AIRFOIL 3
        if Re_w < 98000
           A = 8.06E-8*(RE/1000)^2 - 4.25E-5*(RE/1000)+ 3.05E-2;
           B = 1.48E-6*(RE/1000)^2 - 3.86E-4*(RE/1000)+ 3.06E-3;
           C = 1.22E-6*(RE/1000)^2 - 3.55E-4*(RE/1000)+ 4.09E-2;
           CDP = A*CL^2 + B*CL + C;  %parasite drag coef.
        elseif Re_w < 102000 % goofy fix - an attempt to smooth the discontinuity between drag regimes
           temp = (102000-RE) / 4000;
           e = (1.5 - temp)^4;
             temp = temp^e;
           A = 8.06E-8*(RE/1000)^2 - 4.25E-5*(RE/1000)+ 3.05E-2;
           B1 = 1.48E-6*(RE/1000)^2 - 3.86E-4*(RE/1000)+ 3.06E-3;
           B2 = 6.16E-8*(RE/1000)^2 - 3.60E-5*(RE/1000)- 1.7756E-2;
           B = temp*B1 + (1-temp)*B2;
           C1 = 1.22E-6*(RE/1000)^2 - 3.55E-4*(RE/1000)+ 4.09E-2;
           C2 = 1.66E-7*(RE/1000)^2 - 9.34E-5*(RE/1000)+ 2.528E-2;
           C = temp*C1 + (1-temp)*C2;
           CDP = A*CL^2 + B*CL + C;  %parasite drag coef.
        else
           A = 8.06E-8*(RE/1000)^2 - 4.25E-5*(RE/1000)+ 3.05E-2;
           B = 6.16E-8*(RE/1000)^2 - 3.60E-5*(RE/1000)- 1.7756E-2;
           C = 1.66E-7*(RE/1000)^2 - 9.34E-5*(RE/1000)+ 2.528E-2;
           CDP = A*CL^2 + B*CL + C;  %parasite drag coef.
        end

    elseif airfoil == 5     
        %% AIRFOIL 5 - MH81 airfoil for flying wing
            x = [50000, 100000, 150000, 200000, 250000, 300000];     % Reynolds number
            y = [0.00 0.10 0.20 0.30 0.40 0.50 0.60 0.70 0.75 0.780 0.80 0.90 1.00 1.10 1.20];  % Lift Coefficient
            z1 = load('MH81data_re150e3.txt');
            z1 = load('MH81data_re200e3.txt');
            z4 = load('MH81data_re250e3.txt');
            z5 = load('MH81data_re300e3.txt');
            z(1,:) = [.01929 .01901 .01806 .02035 .02429 .03945 .05 .07 .09 .11 .13 .15 .18 .21 .25];
            z(2,:) = [.01023 .01019 .00929 .00933 .00991 .01111 .01316 .01582 .01744 .01868 .01892 .02290 .02813 0.03336 .03859];
            z(3,:) = z1(:,4)';
            z(4,:) = z1(:,4)';
            z(5,:) = z4(:,4)';
            z(6,:) = z5(:,4)';

            % CAUTION: z(1,13), z(1,14), and z(1,15) are not obtained from XFOIL.  XFOIL
            % could not converge to find these values, so they are guessed. the
            % z(1,13) point is actually from x-foil but for CL = 0.983


            % Pseudo Code
            % (1) Bound CL with y[]
            if (CL < 0.0)
                '!!! WARNING: CL < 0.0'
                return
            elseif (CL > 1.2)
                '!!! WARNING: CL > 1.2'
                return
            end
            i = 1;
            % y(1);
            while CL >= y(i)
                i = i+1;
            end
            CLL = i-1;   % lower bound on CL index
            CLU = i;

            % (2) Bound RE with x[]
            if (RE < 50000) % Approximate lower Reynolds numbers using data for RE=50k
                RE = 50000;
            elseif (RE >= 300000) % Approx. higher RE using data for RE = 300k
                RE = 299995;
            end
    %         disp('(from drag_polar) RE = '); disp(RE);
            i2 = 1;
            while RE >= x(i2)
                i2 = i2+1;
            end
            REL = i2-1;   % lower bound on RE
            REU = i2;

            % (3) Interpolate between CLs to find two CDPs
            CDP_L = z(REL, CLL) + (CL-y(CLL))/(y(CLU)-y(CLL)) * (z(REL, CLU) - z(REL, CLL));    % CDP at lower RE
            CDP_U = z(REU, CLL) + (CL-y(CLL))/(y(CLU)-y(CLL)) * (z(REU, CLU) - z(REU, CLL));    % CDP at higher RE

            % (4) Interpolate between CDPs to find CDP
            CDP = CDP_L + (RE-x(REL))/(x(REU)-x(REL)) * (CDP_U - CDP_L);
            if (CL > 1.2)
                CDP
                pause
            end
    
    elseif airfoil == 6     
        %% AIRFOIL 6 - winglet
            x = [30000, 60000, 100000, 150000];     % Reynolds number
            y = [0.00 0.10 0.20 0.30 0.40 0.50 0.60 0.70 0.80 0.90 1.00 1.10];  % Lift Coefficient
            z1 = load('psu94097data_re30e3(1).dat');
            z1 = load('psu94097data_re60e3(1).dat');
            z1 = load('psu94097data_re100e3(1).dat');
            z4 = load('psu94097data_re150e3(1).dat');
            z(1,1:8) = z1(1:8,4)';  % Parasite drag data
            z(1,9:12) = [0.068, 0.07879, 0.09, 0.1];  % !!! WARNING: THESE VALUES ARE ESTIMATED, XFOIL DID NOT CONVERGE!!!
            z(2,:) = z1(:,4)';  
            z(3,:) = z1(:,4)';  
            z(4,:) = z4(:,4)';

            % Pseudo Code
            % (1) Bound CL with y[]
            if (CL < 0.0)
                '!!! WARNING: CL < 0.0'
                return
            elseif (CL > 1.1)
                '!!! WARNING: CL > 1.1'
                return
            end
            i = 1;
            % y(1);
            while CL >= y(i)
                i = i+1;
            end
            CLL = i-1;   % lower bound on CL index
            CLU = i;

            % (2) Bound RE with x[]
            if (RE < 30000) % Approximate lower Reynolds numbers using data for RE=30k
                disp('Warning: RE < 30000')
                disp('Using RE = 30000 for winglet in drag_polar')
                RE = 30000;
            elseif (RE >= 150000) % Approx. higher RE using data for RE = 150k
                RE = 149995;
                disp('Warning: RE > 150000')
                disp('Using RE = 149995 for winglet in drag_polar')
            end
    %         disp('(from drag_polar) RE = '); disp(RE);
            i2 = 1;
            while RE >= x(i2)
                i2 = i2+1;
            end
            REL = i2-1;   % lower bound on RE
            REU = i2;

            % (3) Interpolate between CLs to find two CDPs
            CDP_L = z(REL, CLL) + (CL-y(CLL))/(y(CLU)-y(CLL)) * (z(REL, CLU) - z(REL, CLL));    % CDP at lower RE
            CDP_U = z(REU, CLL) + (CL-y(CLL))/(y(CLU)-y(CLL)) * (z(REU, CLU) - z(REU, CLL));    % CDP at higher RE

            % (4) Interpolate between CDPs to find CDP
            CDP = CDP_L + (RE-x(REL))/(x(REU)-x(REL)) * (CDP_U - CDP_L);
            if (CL > 1.2)
                CDP
                pause
            end
    elseif airfoil ==7
        %% Airfoil 7 - E 205 RC glider airfoil 
        x = [25000 50000 75000 100000 150000 200000 250000 300000];      % Reynolds Number
        y = [0 .05 .1 .15 .20 .25 .3 .35 .40 .45 .50 .55 .60 .65 .70 .75 .80 .85 .90 .95 1.0]; % Lift Coefficient
        z1 = load('E205RE25k.txt');
        z2 = load('E205RE50k.txt');
        z3 = load('E205RE75k.txt');
        z4 = load('E205RE100k.txt');
        z5 = load('E205RE150k.txt');
        z6 = load('E205RE200k.txt');
        z7 = load('E205RE250k.txt');
        z8 = load('E205RE300k.txt');
        % Grab Drag Coefficients   % Grab Moment Coefficients
        z(1,:) = z1(:,3)';          w(1,:) = z1(:,5)';
        z(2,:) = z2(:,3)';          w(2,:) = z2(:,5)';    
        z(3,:) = z3(:,3)';          w(3,:) = z3(:,5)';
        z(4,:) = z4(:,3)';          w(4,:) = z4(:,5)';
        z(5,:) = z5(:,3)';          w(5,:) = z5(:,5)';
        z(6,:) = z6(:,3)';          w(6,:) = z6(:,5)';
        z(7,:) = z7(:,3)';          w(7,:) = z7(:,5)';
        z(8,:) = z8(:,3)';          w(8,:) = z8(:,5)';
        
        % Pseudo Code
        % (1) Bound CL with y[]
        if (CL < 0.0)
            '!!! WARNING: CL < 0.0'
            return
        elseif (CL > 1.1)
            CL
            '!!! WARNING: CL > 1.1'
            return
        end
        i = 1;
        % y(1);
        while CL >= y(i)
            i = i+1;
        end
        CLL = i-1;   % lower bound on CL index
        CLU = i;

        % (2) Bound RE with x[]
        if (RE < 25000) % Approximate lower Reynolds numbers using data for RE=50k
            disp('Warning: RE < 25000')
            disp('Using RE = 25000 for wing in drag_polar')
            RE = 25000;
        elseif (RE >= 300000) % Approx. higher RE using data for RE = 150k
%             disp('Reynolds Number')
            
            RE = 299995;
%             disp('Warning: RE > 300000')
%             disp('Using RE = 299995 for wing in drag_polar')
        end
%         disp('(from drag_polar) RE = '); disp(RE);
        i2 = 1;
        while RE >= x(i2)
            i2 = i2+1;
        end
        REL = i2-1;  % lower bound on RE
        REU = i2;
        
        % (3) Interpolate between CLs to find two CDPs
        CDP_L = z(REL, CLL) + (CL-y(CLL))/(y(CLU)-y(CLL)) * (z(REL, CLU) - z(REL, CLL));    % CDP at lower RE
        CDP_U = z(REU, CLL) + (CL-y(CLL))/(y(CLU)-y(CLL)) * (z(REU, CLU) - z(REU, CLL));    % CDP at higher RE

        CDM_L = w(REL, CLL) + (CL-y(CLL))/(y(CLU)-y(CLL)) * (w(REL, CLU) - w(REL, CLL));    % CDM at lower RE
        CDM_U = w(REU, CLL) + (CL-y(CLL))/(y(CLU)-y(CLL)) * (w(REU, CLU) - w(REU, CLL));    % CDM at higher RE
        % (4) Interpolate between CDPs to find CDP
        CDP = CDP_L + (RE-x(REL))/(x(REU)-x(REL)) * (CDP_U - CDP_L);
        CM = CDM_L + (RE-x(REL))/(x(REU)-x(REL)) * (CDM_U - CDP_L);
        if (CL > 1.2)
            CDP
            CM
            pause
        end
end






