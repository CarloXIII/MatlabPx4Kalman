%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Writer:        Schuler Carlo                                      %
%    Email:         carlo.schuler@stud.hslu.ch                         %
%    Organisation:  HSLU                                               %
%    Date:          Mai 2014                                           %
%    Version:       V3                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Algemeines Skript zum verifizieren der Paraglider Daten
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear all;
close all;
format compact;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1)   Erklärungen
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  ---------------------------------------------------------------------
%  Variable     ¦   Einheit     ¦  Bezeichnung      ¦ Erläuterung
%  ---------------------------------------------------------------------
%  XIMU_AccX    |   [m/s^2]     |   Acceleration    | XSENS-Beschleunigungssensor X-Achse
%  XIMU_AccY    |   [m/s^2]     |   Acceleration    | XSENS-Beschleunigungssensor Y-Achse
%  XIMU_AccZ    |   [m/s^2]     |   Acceleration    | XSENS-Beschleunigungssensor Z-Achse
%  
%  XIMU_GyroX   |   [m/s^2]     |  Kreiselfrequenz  | XSENS-Kreiselfrequenzen X-Achse
%  XIMU_GyroY   |   [m/s^2]     |  Kreiselfrequenz  | XSENS-Kreiselfrequenzen Y-Achse
%  XIMU_GyroZ   |   [m/s^2]     |  Kreiselfrequenz  | XSENS-Kreiselfrequenzen Z-Achse
%
%  XIMU_MagX    |   [Gauss]     |   Magnitude       | XSENS-Magnetometer X-Achse
%  XIMU_MagY    |   [Gauss]     |   Magnitude       | XSENS-Magnetometer Y-Achse
%  XIMU_MagZ    |   [Gauss]     |   Magnitude       | XSENS-Magnetometer Z-Achse
%
%  XGPS_Lat     |    [deg]      |   GPS-Position    | XSENS-GPS Latitude
%  XGPS_Lon     |    [deg]      |   GPS-Position    | XSENS-GPS Longitude
%  XGPS_Alt     |    [müM]      |   GPS-Position    | XSENS-GPS Altitude
%
%  XGPS_VelN    |    [m/s]      | GPS-Geschwindigk. | XSENS-GPS Geschw.Nord
%  XGPS_VelE    |    [m/s]      | GPS-Geschwindigk. | XSENS-GPS Geschw.East
%  XGPS_VelD    |    [m/s]      | GPS-Geschwindigk. | XSENS-GPS Geschw.Down
%
% XSEN_BaroAlt  |     [m]       |   Baro-Altitude   | XSENS-Druck für Höhe
% XSEN_BaroPres |    [mbar]     |   Baro-Druck      | XSENS-Druck



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2) Wahl der Datenreihe
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
clc;
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TEST-REIHE         |  TEST_NUMBER
%-----------------------------------------
% Flug 1:            |     (1)
% Flug 2:            |     (2)
% KalmanTestAmBoden  |     (3)
% KalmanTestSpaz1    |     (4)
% KalmanTestFeld     |     (5)
% VelTest1           |     (6)
% Flug 3:            |     (7)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
TEST_NUMBER = 7
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 2)  Daten einlesen
% Logging Frequenz 100 Hz
if(TEST_NUMBER == 7)
load('Datenreihen/Flug3.mat')
 % Konstanten
  Datenreihe = 'Flug3'
  CAL_LIMIT_GYRO = 18376 
  CAL_LIMIT_ACC = 18376 
  G_earth = 9.81;
 % Annahme der Initialwinkel
  INITIAL_ROLL_GUESS = 0
  INITIAL_PITCH_GUESS = pi/36
  INITIAL_YAW_GUESS = 0
end
if(TEST_NUMBER == 6)
load('Datenreihen/VelTest1.mat')
Datenreihe = 'VelTest1'
 % Konstanten
  CAL_LIMIT_GYRO = 1428 
  CAL_LIMIT_ACC = 1428
  G_earth = 9.81;
 % Annahme der Initialwinkel
  INITIAL_ROLL_GUESS = 0
  INITIAL_PITCH_GUESS = pi/36
  INITIAL_YAW_GUESS = 0
end
if(TEST_NUMBER == 5)
load('Datenreihen/KalmanTestFeld.mat')
Datenreihe = 'KalmanTestFeld'
 % Konstanten
  CAL_LIMIT_GYRO = 7000 
  CAL_LIMIT_ACC = 7000
  G_earth = 9.81;
 % Annahme der Initialwinkel
  INITIAL_ROLL_GUESS = -0.04
  INITIAL_PITCH_GUESS = pi/36
  INITIAL_YAW_GUESS = 0
end
if(TEST_NUMBER == 4)
load('Datenreihen/KalmanTestSpaz1.mat')
Datenreihe = 'KalmanTestSpaz1'
 % Konstanten
  CAL_LIMIT_GYRO = 35000 
  CAL_LIMIT_ACC = 35000
  G_earth = 9.81;
 % Annahme der Initialwinkel
  INITIAL_ROLL_GUESS = 0
  INITIAL_PITCH_GUESS = pi/36
  INITIAL_YAW_GUESS = 0
end
if(TEST_NUMBER == 3)
load('Datenreihen/KalmanTestAmBoden.mat')
Datenreihe = 'KalmanTestAmBoden'
 % Konstanten
  CAL_LIMIT_GYRO = 1321 
  CAL_LIMIT_ACC = 1321
  G_earth = 9.81;
 % Annahme der Initialwinkel
  INITIAL_ROLL_GUESS = 0
  INITIAL_PITCH_GUESS = pi/36
  INITIAL_YAW_GUESS = 0
end
if(TEST_NUMBER == 2)
load('Datenreihen/Flug2.mat')
Datenreihe = 'Flug2'
 % Konstanten
  CAL_LIMIT_GYRO = 17818 
  CAL_LIMIT_ACC = 17818 
  G_earth = 9.81;
 % Annahme der Initialwinkel
  INITIAL_ROLL_GUESS = 0
  INITIAL_PITCH_GUESS = pi/36
  INITIAL_YAW_GUESS = 0
end
if(TEST_NUMBER == 1)
load('Datenreihen/Flug1.mat')
Datenreihe = 'Flug1'
 % Konstanten
  CAL_LIMIT_GYRO = 17881 
  CAL_LIMIT_ACC = 17881
  G_earth = 9.81;
 % Annahme der Initialwinkel
  INITIAL_ROLL_GUESS = 0
  INITIAL_PITCH_GUESS = pi/36
  INITIAL_YAW_GUESS = 0
end
        
 
% 3) Zeitstempel der Sensordaten
 % Beschleunigung, Kreiselbewegung, Kompass
    IMU_Time = IMU_Time-(min(IMU_Time)); IMU_Time = IMU_Time/(10^6);
    IMU_Time = IMU_Time-(min(IMU_Time)); IMU_Time = IMU_Time/(10^6);
 % GPS-Daten (Positions/Geschwindigkeits Zeitstempel
    XGPS_POS_t = XGPS_POS_t-(min(XGPS_POS_t)); XGPS_POS_t = XGPS_POS_t/10^6;
    XGPS_VEL_t = XGPS_VEL_t-(min(XGPS_VEL_t)); XGPS_VEL_t = XGPS_VEL_t/10^6;
 % Baro-Daten
    TIME_StartTime = TIME_StartTime - (min(TIME_StartTime)); TIME_StartTime = TIME_StartTime / 10^6;

    % Umrechnungsfaktor rat2deg
    R2D = (360/(2*pi));
    
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3) Accelerometer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %Plot XYZ
    
    figure(1);
    
    set(gcf,'name',sprintf('3) Accelerometer (Datenreihe: %s)', Datenreihe) ,'numbertitle','off')
    
    ax(1) = subplot(3,1,1);
    plot(IMU_Time, [XIMU_AccX,IMU_AccX])
    legend('Xsens','Px4'); grid on; title({'Rohwerte der Beschleunigungssensoren','X-Achse'},'FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Beschleunigung [m/s^2]');
    
    ax(2) = subplot(3,1,2);
    plot(IMU_Time, [XIMU_AccY,IMU_AccY])
    legend('Xsens','Px4'); grid on; title('Y-Achse','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Beschleunigung [m/s^2]');
    
    ax(3) = subplot(3,1,3);
    plot(IMU_Time, [XIMU_AccZ,IMU_AccZ])
    legend('Xsens','Px4'); grid on; title('Z-Achse','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Beschleunigung [m/s^2]');

   
    linkaxes(ax,'x');

    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4) Gyros
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
    %Plot XYZ
    figure(2)
    set(gcf,'name',sprintf('4) Gyros (Datenreihe: %s)',Datenreihe),'numbertitle','off')
    
    ax(1) = subplot(3,1,1);
    plot(IMU_Time, [XIMU_GyroX,IMU_GyroX])
    legend('Xsens','Px4'); grid on; title({'Rohwerte der Gyros','X-Achse'},'FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Kreiselfrequenz [rad/s]');
    
    ax(2) = subplot(3,1,2);
    plot(IMU_Time, [XIMU_GyroY,IMU_GyroY])
    legend('Xsens','Px4'); grid on; title('Y-Achse','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Kreiselfrequenz [rad/s]');
    
    ax(3) = subplot(3,1,3);
    plot(IMU_Time, [XIMU_GyroZ, IMU_GyroZ])
    legend('Xsens','Px4'); grid on; title('Z-Achse','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Kreiselfrequenz [rad/s]');

    linkaxes(ax,'x');
    
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%% 5) Magnetometer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %Plot XYZ
    figure(3)
    set(gcf,'name',sprintf('5) Magnetometer (Datenreihe: %s)',Datenreihe),'numbertitle','off')
    ax(1) = subplot(3,1,1);
    plot(IMU_Time, [XIMU_MagX,IMU_MagX])
    legend('Xsens MagX', 'Px4'); grid on; title('GyroX','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Magnitude [Gauss]');
    
    ax(2) = subplot(3,1,2);
    plot(IMU_Time, [XIMU_MagZ,IMU_MagZ])
    legend('Xsens MagY', 'Px4'); grid on; title('GyroY','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Magnitude [Gauss]');
    
    ax(3) = subplot(3,1,3);
    plot(IMU_Time, [XIMU_MagZ,IMU_MagZ])
    legend('Xsens MagZ', 'Px4'); grid on; title('GyroZ','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Magnitude [Gauss]');

    linkaxes(ax,'x');
   
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 6) Baro-Daten
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    figure(4)
    set(gcf,'name',sprintf('6) Baro-Daten (Datenreihe: %s)',Datenreihe),'numbertitle','off')
    ax(1) = subplot(2,1,1);
    plot(TIME_StartTime, [XSEN_BaroPres, SENS_BaroPres])
    legend('Xsens Baro','Px4 Baro'); grid on; title('Baro Druck','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Druck [mbar]');
    
    ax(2) = subplot(2,1,2);
    plot(TIME_StartTime, [XSEN_BaroAlt,SENS_BaroAlt])
    legend('Xsens Baro','Px4 Baro'); grid on; title('Baro Altitude','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Höhe [m]');
    linkaxes(ax,'x')
 
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 7) GPS (Lat/Lon/Alt)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
   %Plot Lat / Lon / Alt
    figure(5)
    set(gcf,'name',sprintf('7) GPS (Lat/Lon/Alt) (Datenreihe: %s)',Datenreihe),'numbertitle','off')
    ax(1) = subplot(3,1,1);
    plot(XGPS_POS_t, XGPS_Lat)
    legend('Xsens GPS'); grid on; title('Latitude','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Grad [°]');
    
    ax(2) = subplot(3,1,2);
    plot(XGPS_POS_t, XGPS_Lon)
    legend('Xsens GPS'); grid on; title('Longitude','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Grad [°]');
    
    ax(3) = subplot(3,1,3);
    plot(XGPS_POS_t, XGPS_Alt)
    legend('Xsens GPS'); grid on; title('Altitude','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Höhe [müM]');

    linkaxes(ax,'x');
    
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%% 8) Winkel Pitch / Yaw / Roll (Kalman Openloop)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 %Berechnen der Winkel mittels XSENS-Sensoren
 %------------------------------------------------------

 % Erstellen der GyroMatrix(zur einfacheren Rechnung)
 GyroMatX = [XIMU_GyroX';...
             XIMU_GyroY';...
             XIMU_GyroZ'];
 
 % Bereichnen der Gyro-Offset-Matrix
 GyroMatX_Off = [mean(XIMU_GyroX(1:CAL_LIMIT_GYRO));...
                 mean(XIMU_GyroY(1:CAL_LIMIT_GYRO));...
                 mean(XIMU_GyroZ(1:CAL_LIMIT_GYRO))]
%GyroMatX_Off = [-0.0066; 0.0059; 0.0078] % Nach BDA von Orfeo Castelletti

AngMatX = zeros(3,length(TIME_StartTime));  



AngMatX(1,1) = INITIAL_ROLL_GUESS;
AngMatX(2,1) = INITIAL_PITCH_GUESS;
AngMatX(3,1) = INITIAL_YAW_GUESS;


% Berechnen der Winkel   
    for i=1:1:(length(TIME_StartTime)-1)
        phi = AngMatX(1,i);
        theta = AngMatX(2,i);
        psi = AngMatX(3,i);
        
        % Tw
        Tw11 = 1;
        Tw21 = 0;
        Tw31 = 0;
        Tw12 = (sin(phi)*sin(theta))/(cos(theta));
        Tw22 = cos(phi);
        Tw32 = sin(phi)/(cos(theta));
        Tw13 = (cos(phi)*sin(theta))/(cos(theta));
        Tw23 = -sin(phi);
        Tw33 = cos(phi)/(cos(theta));
        Tw = [Tw11 Tw12 Tw13; Tw21 Tw22 Tw23; Tw31 Tw32 Tw33];
        
        % Integrieren der Gyros
        dt = TIME_StartTime(i+1)-TIME_StartTime(i);
        AngMatX(:,(i+1)) = AngMatX(:,i)+Tw*(GyroMatX(:,i)-GyroMatX_Off).*dt;
    end


  
    
% Berechnen der Cb-Matrix
%----------------------------------------------------------

InitAngle_Roll = mean(AngMatX(1,(2:CAL_LIMIT_GYRO)));
InitAngle_Pitch = mean(AngMatX(2,(2:CAL_LIMIT_GYRO)));
InitAngle_Yaw = mean(AngMatX(3,(2:CAL_LIMIT_GYRO)));
 
sr=sin(InitAngle_Roll);
cr=cos(InitAngle_Roll);
sp=sin(InitAngle_Pitch);
cp=cos(InitAngle_Pitch);
sy=sin(InitAngle_Yaw);
cy=cos(InitAngle_Yaw);
   
Cb11 = cp*cy;
Cb21 = cp*sy;
Cb31 = -sp;
Cb12 = -cr*sy+sr*sp*cy;
Cb22 = cr*cy+sr*sp*sy; 
Cb32 = sr*cp; 
Cb13 = sr*sy+cr*sp*cy;
Cb23 = -sr*cy+cr*sp*sy;
Cb33 = cr*cp;
Cb_bn = [Cb11 Cb12 Cb13; Cb21 Cb22 Cb23; Cb31 Cb32 Cb33];

%Offset Werte der Beschleunigungen im Ruhezustand
xsens_minBodyAcc = [mean(XIMU_AccX(1:CAL_LIMIT_ACC));...
                    mean(XIMU_AccY(1:CAL_LIMIT_ACC));...
                    mean(XIMU_AccZ(1:CAL_LIMIT_ACC))];
          
px4_minBodyAcc = [mean(IMU_AccX(1:CAL_LIMIT_ACC));...
                  mean(IMU_AccY(1:CAL_LIMIT_ACC));...
                  mean(IMU_AccZ(1:CAL_LIMIT_ACC))];          

g_vec = [0; 0; G_earth];
 
% Beschleunigungs-Offsetvector XSENS
xsens_AccOffs_vec = xsens_minBodyAcc-(-transpose(Cb_bn))*g_vec
%ns_AccOffs_vec = [-0.0126; 0.0159; 0]
%xsens_AccOffs_vec = [-0.7953; 0.014281; -0.1179]


% Beschleunigungs-Offsetvector PX4
px4_AccOffs_vec = px4_minBodyAcc-(-transpose(Cb_bn))*g_vec
        
% (Initialwinkel)
fx_b_mean = mean(XIMU_AccX(1:CAL_LIMIT_ACC))-xsens_AccOffs_vec(1);
fy_b_mean = mean(XIMU_AccY(1:CAL_LIMIT_ACC))-xsens_AccOffs_vec(2);
fz_b_mean = mean(XIMU_AccZ(1:CAL_LIMIT_ACC))-xsens_AccOffs_vec(3);
INITIAL_ROLL = atan2(-fy_b_mean,-fz_b_mean)
INITIAL_PITCH = atan2(fx_b_mean,(sqrt(fy_b_mean^2+fz_b_mean^2)))
INITIAL_YAW = 0
        
    
% Winkel mittels XSENS-Sensoren (AngMatX)
%------------------------------------------------------
 DETREND = 0;
% Erstellen der GyroMatrix(zur einfacheren Rechnung)
 GyroMatX = [XIMU_GyroX';...
             XIMU_GyroY';...
             XIMU_GyroZ'];
 
% Bereichnen der Gyro-Offset-Matrix
GyroMatX_Off = [mean(XIMU_GyroX(1:CAL_LIMIT_GYRO));...
                mean(XIMU_GyroY(1:CAL_LIMIT_GYRO));...
                mean(XIMU_GyroZ(1:CAL_LIMIT_GYRO))];
 
AngMatX = zeros(3,length(TIME_StartTime));

% (Initialwinkel)
AngMatX(1,1) = INITIAL_ROLL;
AngMatX(2,1) = INITIAL_PITCH;
AngMatX(3,1) = INITIAL_YAW;

% Berechnen der Winkel   
    for i=1:1:(length(TIME_StartTime)-1)
        phi = AngMatX(1,i);
        theta = AngMatX(2,i);
        psi = AngMatX(3,i);
        
        % Tw
        Tw11 = 1;
        Tw21 = 0;
        Tw31 = 0;
        Tw12 = (sin(phi)*sin(theta))/(cos(theta));
        Tw22 = cos(phi);
        Tw32 = sin(phi)/(cos(theta));
        Tw13 = (cos(phi)*sin(theta))/(cos(theta));
        Tw23 = -sin(phi);
        Tw33 = cos(phi)/(cos(theta));
        Tw = [Tw11 Tw12 Tw13; Tw21 Tw22 Tw23; Tw31 Tw32 Tw33];
        
        % Integrieren der Gyros
        dt = TIME_StartTime(i+1)-TIME_StartTime(i);
        AngMatX(:,(i+1)) = AngMatX(:,i)+Tw*(GyroMatX(:,i)-GyroMatX_Off).*dt;
    end

    
% Plots
%------------------------------------------------
figure(6);
set(gcf,'name',sprintf('8) Winkel Pitch / Yaw / Roll (Kalman u. Openloop) (Datenreihe: %s)',Datenreihe),'numbertitle','off')
    ax(1) = subplot(3,1,1);
    plot(TIME_StartTime, [ATT_Roll, XATT_Roll]); hold on; plot(TIME_StartTime', AngMatX(1,:),':c')
    legend('Px4 (Kalman)', 'Xsens (Kalman)','Berechnet (OpenLoop)'); grid on; title('Rollwinkel','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    ax(2) = subplot(3,1,2);
    plot(TIME_StartTime, [(ATT_Pitch),XATT_Pitch]);hold on; plot(TIME_StartTime', AngMatX(2,:),':c')
    legend('Px4 (Kalman)', 'Xsens (Kalman)','Berechnet (OpenLoop)'); grid on; title('Pitchwinkel','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    ax(3) = subplot(3,1,3);
    plot(TIME_StartTime, [unwrap(ATT_Yaw), unwrap(XATT_Yaw)]); hold on; plot(TIME_StartTime', AngMatX(3,:),':c')
    legend('Px4 (Kalman)', 'Xsens (Kalman)','Berechnet (OpenLoop)'); grid on; title('Yawwinkel','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    linkaxes(ax,'x');
    
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%% 9) Höhen-Vergleich (Baro,GPS,Kalman)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

   figure(7);
    set(gcf,'name',sprintf('9) Höhen-Vergleich (Baro,GPS,Kalman) (Datenreihe: %s)',Datenreihe),'numbertitle','off')
    plot(TIME_StartTime, [XGPS_Alt-XGPS_Alt(1),XSEN_BaroAlt-XSEN_BaroAlt(1),XGPO_Alt-XGPO_Alt(1),SENS_BaroAlt-SENS_BaroAlt(1), GPOS_Alt-GPOS_Alt(1)]); grid on;
    legend('XSENS GPS','XSENS Baro','XSENS Kalmann','PX4 Baro', 'PX4 Kalmann','PALT'); grid on;  title('Höhenvergleich (Baro, GPS, Kalman)','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Höhe [m]');
    
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%% 10) Acc/Geschw (NED) - Berechnet OpenLoop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Berechnungen
%--------------------------------------

  % Erstellen der XSENS Acc-Matrix(zur einfacheren Rechnung)  
   xsens_AccMat = [XIMU_AccX';...
                   XIMU_AccY';...
                   XIMU_AccZ'];           
   xsens_AccMat_ned = zeros(3,length(TIME_StartTime));

   
   % Erstellen der XSENS Acc-Matrix(zur einfacheren Rechnung)  
   px4_AccMat = [IMU_AccX';...
                   IMU_AccY';...
                   IMU_AccZ'];           
   px4_AccMat_ned = zeros(3,length(TIME_StartTime));

   % Berechnen der Beschleunigungen in NED (xsens_AccMat_ned, px4_AccMat_ned)
   for i=1:1:(length(TIME_StartTime)-1)
        roll  = AngMatX(1,i);
        pitch = AngMatX(2,i);
        yaw  =  AngMatX(3,i);

        % Berechnen der Cb-Matrix
        sr=sin(roll);
        cr=cos(roll);
        sp=sin(pitch);
        cp=cos(pitch);
        sy=sin(yaw);
        cy=cos(yaw);
   
        Cb11 = cp*cy;
        Cb21 = cp*sy;
        Cb31 = -sp;
        Cb12 = -cr*sy+sr*sp*cy;
        Cb22 = cr*cy+sr*sp*sy; 
        Cb32 = sr*cp; 
        Cb13 = sr*sy+cr*sp*cy;
        Cb23 = -sr*cy+cr*sp*sy;
        Cb33 = cr*cp;
        Cb_bn = [Cb11 Cb12 Cb13; Cb21 Cb22 Cb23; Cb31 Cb32 Cb33];
        g_vec = [0; 0; G_earth];
        

        
        
        % XSENS Beschleunigungen
        xsens_AccMat_ned(:,i) = Cb_bn*(xsens_AccMat(:,i)-xsens_AccOffs_vec)+g_vec;%
        % PX4 Beschleunigungen
        px4_AccMat_ned(:,i) = Cb_bn*(px4_AccMat(:,i)-px4_AccOffs_vec)+g_vec;
   end

   
% Plots
%----------------------------------------------
   
   % Plot der Beschleunigungen im BodyFrame
   figure(8);
   set(gcf,'name',sprintf('10) Acc/Geschw (NED) - Berechnet OpenLoop (Datenreihe: %s)',Datenreihe),'numbertitle','off')
   gx(1)=subplot(3,1,1);
   plot(TIME_StartTime, xsens_AccMat(1,:)); hold on;  plot(TIME_StartTime, px4_AccMat(1,:),'r');
   legend('Xsens','Px4'); grid on; title('Beschleunigung X','FontWeight','bold');
   xlabel('Zeitvektor [s]'); ylabel('Beschl [m/s^2]');
   
   gx(2)=subplot(3,1,2);
   plot(TIME_StartTime, xsens_AccMat(2,:)); hold on;  plot(TIME_StartTime, px4_AccMat(2,:),'r');
   legend('Xsens','Px4'); grid on; title('Beschleunigung Y','FontWeight','bold');
   xlabel('Zeitvektor [s]'); ylabel('Beschl [m/s^2]');
   
   gx(3)=subplot(3,1,3);
   plot(TIME_StartTime, xsens_AccMat(3,:)); hold on;  plot(TIME_StartTime, px4_AccMat(3,:),'r');
   legend('Xsens','Px4'); grid on; title('Beschleunigung Z','FontWeight','bold');
   xlabel('Zeitvektor [s]'); ylabel('Beschl [m/s^2]');
   linkaxes(gx,'x');
   
   
   % Plot der Beschleunigungen im Navigation-Frame
   figure(9);
   set(gcf,'name',sprintf('10) Acc/Geschw (NED) - Berechnet OpenLoop (Datenreihe: %s)',Datenreihe),'numbertitle','off')
   bx(1)=subplot(3,1,1);
   plot(TIME_StartTime, xsens_AccMat_ned(1,:)); hold on;  plot(TIME_StartTime, px4_AccMat_ned(1,:),'r');
   legend('Xsens','Px4'); grid on; title('Beschleunigung North','FontWeight','bold');
   xlabel('Zeitvektor [s]'); ylabel('Beschl [m/s^2]');
   
   bx(2)=subplot(3,1,2);
   plot(TIME_StartTime, xsens_AccMat_ned(2,:)); hold on;  plot(TIME_StartTime, px4_AccMat_ned(2,:),'r');
   legend('Xsens','Px4'); grid on; title('Beschleunigung East','FontWeight','bold');
   xlabel('Zeitvektor [s]'); ylabel('Beschl [m/s^2]');
   
   bx(3)=subplot(3,1,3);
   plot(TIME_StartTime, xsens_AccMat_ned(3,:)); hold on;  plot(TIME_StartTime, px4_AccMat_ned(3,:),'r');
   legend('Xsens','Px4'); grid on; title('Beschleunigung Down','FontWeight','bold');
   xlabel('Zeitvektor [s]'); ylabel('Beschl [m/s^2]');
   linkaxes(bx,'x');
   
   
% Erstellen der Geschwindigkeitsvektoren
xsens_VelN = zeros(1,length(TIME_StartTime));
xsens_VelE = zeros(1,length(TIME_StartTime));
xsens_VelD = zeros(1,length(TIME_StartTime));

px4_VelN = zeros(1,length(TIME_StartTime));
px4_VelE = zeros(1,length(TIME_StartTime));
px4_VelD = zeros(1,length(TIME_StartTime));

% Berechnen der Geschwindigkeiten durch Integrieren der Beschl. NED

for i=1:1:(length(TIME_StartTime)-1)
    % Xsens
    dt_x = IMU_Time(i+1)-IMU_Time(i);
    
    xsens_VelN(i+1) = xsens_VelN(i) + xsens_AccMat_ned(1,(i+1))*dt_x;
    xsens_VelE(i+1) = xsens_VelE(i) + xsens_AccMat_ned(2,(i+1))*dt_x;
    xsens_VelD(i+1) = xsens_VelD(i) + xsens_AccMat_ned(3,(i+1))*dt_x;
    
    % PX4
    dt_p = IMU_Time(i+1)-IMU_Time(i);
   
    px4_VelN(i+1) = px4_VelN(i) + px4_AccMat_ned(1,(i+1))*dt_p;
    px4_VelE(i+1) = px4_VelE(i) + px4_AccMat_ned(2,(i+1))*dt_p;
    px4_VelD(i+1) = px4_VelD(i) + px4_AccMat_ned(3,(i+1))*dt_p; 
    
end

% PLOTS
%----------------------------------

% Plot All
    figure(10)
    set(gcf,'name',sprintf('10) Acc/Geschw (NED) - Berechnet OpenLoop (Datenreihe: %s)',Datenreihe),'numbertitle','off')
    gx(1) = subplot(3,1,1);
    
    plot(TIME_StartTime,xsens_VelN); hold on; 
    plot(TIME_StartTime,px4_VelN,'r'); hold on;
    plot(TIME_StartTime,XGPS_VelD,'g'); grid on;
    xlabel('Zeitvektor [s]'); ylabel('Geschw. [m/s]'); 
    title('Geschwindigkeit NORTH','FontWeight','bold');
    legend('v_N XSENS (Berechnet)','v_N Px4 (Berechnet)','v_N GPS');
    
    % Plot East
    gx(2) = subplot(3,1,2);
    plot(TIME_StartTime,xsens_VelE); hold on; 
    plot(TIME_StartTime,px4_VelE,'r'); hold on;
    plot(TIME_StartTime,XGPS_VelE,'g'); grid on;
    title('Geschwindigkeit EAST','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Geschw. [m/s]'); 
    legend('v_E XSENS (Berechnet)','v_E Px4 (Berechnet)','v_E GPS');
    
    % Plot Down
    gx(3) = subplot(3,1,3);
    plot(TIME_StartTime,xsens_VelD); hold on; 
    plot(TIME_StartTime,px4_VelD,'r'); hold on;
    plot(TIME_StartTime,XGPS_VelD,'g'); grid on;
    title('Geschwindigkeit DOWN','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Geschw. [m/s]'); 
    legend('v_D XSENS (Berechnet)','v_D Px4 (Berechnet)','v_D GPS');
    linkaxes(gx,'x');

    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%% 11) VelNED Vergleich (GPS, Kalman, Berechnet)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% nAn durch 0 ersetzen        
    GPOS_VelN(isnan(GPOS_VelN)) = 0 ;
    GPOS_VelE(isnan(GPOS_VelE)) = 0 ;
    GPOS_VelD(isnan(GPOS_VelD)) = 0 ;
 
 % Plot Down
    figure(11);
    set(gcf,'name',sprintf('11) VelNED Vergleich (GPS, Kalman, Berechnet) (Datenreihe: %s)',Datenreihe),'numbertitle','off')
    cx(1) = subplot(2,1,1);
    plot(IMU_Time, [xsens_VelN])
    legend('Berechnet'); grid on; title('North Berechnet ','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Geschw [m/s]');
    
    cx(2) = subplot(2,1,2);
    plot(IMU_Time, [GPOS_VelN, XGPS_VelN, XGPO_VelN])
    legend('Px4','GPS','Xsens'); grid on; title('North Gemessen','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Geschw [m/s]');
    linkaxes(cx,'x');
 
 % Plot East      
    figure(12);
    set(gcf,'name',sprintf('11) VelNED Vergleich (GPS, Kalman, Berechnet) (Datenreihe: %s)',Datenreihe),'numbertitle','off')
    
    bx(1) = subplot(2,1,1);
    plot(IMU_Time, [xsens_VelE'])
    legend('Berechnet'); grid on; title('East Berechnet','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Geschw [m/s]');

    bx(2) = subplot(2,1,2);
    plot(IMU_Time, [GPOS_VelE, XGPS_VelE, XGPO_VelE])
    legend('Px4','GPS','Xsens'); grid on; title('East Gemessen','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Geschw [m/s]');
    linkaxes(bx,'x');
    
 % Plot Down        
    figure(13);
    set(gcf,'name',sprintf('11) VelNED Vergleich (GPS, Kalman, Berechnet) (Datenreihe: %s)',Datenreihe),'numbertitle','off')
    
    ax(1) = subplot(2,1,1);
    plot(IMU_Time, [xsens_VelD'])
    legend('Berechnet'); grid on; title('Down Berechnet','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Geschw [m/s]');
    
    ax(2) = subplot(2,1,2);
    plot(IMU_Time, [GPOS_VelD, XGPS_VelD, XGPO_VelD])
    legend('Px4','GPS','Xsens'); grid on; title('Down Gemessen','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Geschw [m/s]');
    linkaxes(ax,'x');
    

%% 12) Heading mittels Magnetometer

% Berechnungen
%--------------------------------------

  % Erstellen der XSENS Mag-Matrix(zur einfacheren Rechnung)  
   xsens_MagMat = [XIMU_MagX';...
                   XIMU_MagY';...
                   XIMU_MagZ'];           
   xsens_MagMat_ned = zeros(3,length(TIME_StartTime));
   
   % Erstellen der XSENS Mag-Matrix(zur einfacheren Rechnung)  
   px4_MagMat = [IMU_MagX';...
                 IMU_MagY';...
                 IMU_MagZ'];           
   px4_MagMat_ned = zeros(3,length(TIME_StartTime));
 
   % Berechnen der Magnituden in NED (xsens_MagMat_ned, px4_MagMat_ned)
   for i=1:1:(length(TIME_StartTime)-1)
        roll  = AngMatX(1,i);
        pitch = AngMatX(2,i);
        yaw  =  0;%AngMatX(3,i);

        % Berechnen der Cb-Matrix
        sr=sin(roll);
        cr=cos(roll);
        sp=sin(pitch);
        cp=cos(pitch);
        sy=sin(yaw);
        cy=cos(yaw);
   
        Cb11 = cp*cy;
        Cb21 = cp*sy;
        Cb31 = -sp;
        Cb12 = -cr*sy+sr*sp*cy;
        Cb22 = cr*cy+sr*sp*sy; 
        Cb32 = sr*cp; 
        Cb13 = sr*sy+cr*sp*cy;
        Cb23 = -sr*cy+cr*sp*sy;
        Cb33 = cr*cp;
        Cb_bn = [Cb11 Cb12 Cb13; Cb21 Cb22 Cb23; Cb31 Cb32 Cb33];

        % XSENS Magnituden
        xsens_MagMat_ned(:,i) = Cb_bn*(xsens_MagMat(:,i));
        % PX4 Magnituden
        px4_MagMat_ned(:,i) = Cb_bn*(px4_MagMat(:,i));
 
   end
 
   %Kontrolle ob Geometrische Addition konstant (In Nav und
   % Body-Frame
   xsens_MagMat_konst = [sqrt(xsens_MagMat(1,:).^2+xsens_MagMat(2,:).^2+xsens_MagMat(3,:).^2);...
                        sqrt(xsens_MagMat_ned(1,:).^2+xsens_MagMat_ned(2,:).^2+xsens_MagMat_ned(3,:).^2)];
                    
   px4_MagMat_konst = [sqrt(px4_MagMat(1,:).^2+px4_MagMat(2,:).^2+px4_MagMat(3,:).^2);...
                        sqrt(px4_MagMat_ned(1,:).^2+px4_MagMat_ned(2,:).^2+px4_MagMat_ned(3,:).^2)];
                    
   
                    
   xsens_heading = atan2(-xsens_MagMat_ned(2,:),xsens_MagMat_ned(1,:));
   xsens_heading = xsens_heading.*(360/(2*pi));
   
   px4_heading = atan2(-px4_MagMat_ned(2,:),px4_MagMat_ned(1,:));
   px4_heading = px4_heading.*(360/(2*pi));
   
% Plots
%----------------------------------------------
   % PLOT alles gemeinsam
   figure(14);
   set(gcf,'name',sprintf('12) Heading mittels Magnetometer (Datenreihe: %s)',Datenreihe),'numbertitle','off')
   ax(2) = subplot(2,1,1);
   plot(TIME_StartTime, xsens_MagMat_konst(1,:),'r'); hold on; grid on;
   title('Betrag der Magnetometer Komponenten','FontWeight','bold');
   xlabel('Zeitvektor [s]'); ylabel('Norm');
   ax(1) = subplot(2,1,2); 
   plot(TIME_StartTime, xsens_MagMat_ned(1,:),'b'); hold on;  
   plot(TIME_StartTime, xsens_MagMat_ned(2,:),'g'); hold on; 
   plot(TIME_StartTime, xsens_MagMat_ned(3,:),'m'); hold on; 
   legend('MagX','MagY','MagZ','Betrag'); grid on; title('Magnetometer NED Kompnenten','FontWeight','bold');
   xlabel('Zeitvektor [s]'); ylabel('Norm');
   linkaxes(ax,'xy')

   % Plot der Headings
   figure(15);
   set(gcf,'name',sprintf('12) Heading mittels Magnetometer (Datenreihe: %s)',Datenreihe),'numbertitle','off')
   plot(TIME_StartTime, xsens_heading); hold on; plot(TIME_StartTime, px4_heading,'r');
   legend(); grid on; title('Heading XSENS','FontWeight','bold');
   legend('Xsens','px4');
   xlabel('Zeitvektor [s]'); ylabel('..');
   
%% 13) Motoren / Servo Ansteuerung

figure(16);
   set(gcf,'name',sprintf('13) Motoren / Servo Ansteuerung (Datenreihe: %s)',Datenreihe),'numbertitle','off')
   ax(5) = subplot(2,1,1);
   line([1 length(TIME_StartTime)], [1088 1088],'Color','r','LineStyle',':'); hold on;
   plot(TIME_StartTime, [OUT0_Out3, OUT0_Out4]); grid on;
   legend('Ruhe-PWM (Stillstand)','Motor Links', 'Motor Rechts');
   title({'Motoren / Servo Ansteuerung','Schubmotoren'},'FontWeight','bold');
   xlabel('Zeitvektor [s]'); ylabel('PWM-Ausgang');
   ax(6) = subplot(2,1,2); 
   line([1 length(TIME_StartTime)], [1517 1517],'Color','r','LineStyle',':'); hold on;
   plot(TIME_StartTime, [OUT0_Out1, OUT0_Out2]); hold on;  
   legend('Ruhe-PWM (Ausgangslage)','Servo Links', 'Servo Rechts'); grid on; title('Bremsleinen-Servos','FontWeight','bold');
   xlabel('Zeitvektor [s]'); ylabel('PWM-Ausgang');
   linkaxes(ax,'x')
   
   %% 14) Streck durch Geschwindigkeiten
   
   