%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Writer:        Schuler Carlo                                      %
%    Email:         carlo.schuler@stud.hslu.ch                         %
%    Organisation:  HSLU                                               %
%    Date:          Mai 2014                                           %
%    Version:       V1                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Algemeines Skript zum Plotten wichtiger Kalman-Simulations-Daten
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global TEST_NUMBER
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1) Wahl der Datenreihe
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%clear all;
%clc;
%close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TEST-REIHE         |  TEST_NUMBER
%-----------------------------------------
% Flug 1:            |     (1)
% Flug 2:            |     (2)
% KalmanTestSpaz1    |     (3)
% KalmanTestFeld     |     (4)
% Flug 3:            |     (5)
% GyroTest:          |     (6)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%TEST_NUMBER = 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2)  Daten einlesen
% Logging Frequenz 100 Hz
if(TEST_NUMBER == 6)
    load('Datenreihen\GyroTest_sim');
 % Konstanten
  CAL_LIMIT_GYRO = 100 
  CAL_LIMIT_ACC = 100 
  G_earth = 9.81;
 % Annahme der Initialwinkel
  INITIAL_ROLL_GUESS = 0
  INITIAL_PITCH_GUESS = pi/36
  INITIAL_YAW_GUESS = 0
end
if(TEST_NUMBER == 5)
    load('Datenreihen\Flug3_sim');
 % Konstanten
  CAL_LIMIT_GYRO = 18376 
  CAL_LIMIT_ACC = 18376 
  G_earth = 9.81;
 % Annahme der Initialwinkel
  INITIAL_ROLL_GUESS = 0
  INITIAL_PITCH_GUESS = pi/36
  INITIAL_YAW_GUESS = 0
end
if(TEST_NUMBER == 4)
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
if(TEST_NUMBER == 3)
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
if(TEST_NUMBER == 2)
Datenreihe = 'Flug2'
load('Datenreihen\Flug2_sim');
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
    XIMU_Time = XIMU_Time-(min(XIMU_Time)); XIMU_Time = XIMU_Time/(10^6);
    IMU_Time = IMU_Time-(min(IMU_Time)); IMU_Time = IMU_Time/(10^6);
 % GPS-Daten (Positions/Geschwindigkeits Zeitstempel
    XGPS_POS_t = XGPS_POS_t-(min(XGPS_POS_t)); XGPS_POS_t = XGPS_POS_t/10^6;
    XGPS_VEL_t = XGPS_VEL_t-(min(XGPS_VEL_t)); XGPS_VEL_t = XGPS_VEL_t/10^6;
 % Baro-Daten
    TIME_StartTime = TIME_StartTime - (min(TIME_StartTime)); TIME_StartTime = TIME_StartTime / 10^6;

    % Umrechnungsfaktor rat2deg
    R2D = (360/(2*pi));
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%% 2) Winkel Pitch / Yaw / Roll (Kalman Openloop)
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
figure(1);
set(gcf,'name',sprintf('8) Winkel Pitch / Yaw / Roll (Kalman u. Openloop) (Datenreihe: %s)',Datenreihe),'numbertitle','off')
    ax(1) = subplot(3,1,1);
    plot(TIME_StartTime, [sim_ATT_Roll,ATT_Roll, XATT_Roll]); hold on; plot(TIME_StartTime', AngMatX(1,:),':c')
    legend('Kalman simuliert','Px4 (Kalman)', 'Xsens (Kalman)','Berechnet (OpenLoop)'); grid on; title('Rollwinkel','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    ax(2) = subplot(3,1,2);
    plot(TIME_StartTime, [sim_ATT_Pitch,(ATT_Pitch),XATT_Pitch]);hold on; plot(TIME_StartTime', AngMatX(2,:),':c')
    legend('Kalman simuliert','Px4 (Kalman)', 'Xsens (Kalman)','Berechnet (OpenLoop)'); grid on; title('Pitchwinkel','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    ax(3) = subplot(3,1,3);
    plot(TIME_StartTime, [unwrap(sim_ATT_Yaw),unwrap(ATT_Yaw), unwrap(XATT_Yaw)]); hold on; plot(TIME_StartTime', AngMatX(3,:),':c')
    legend('Kalman simuliert','Px4 (Kalman)', 'Xsens (Kalman)','Berechnet (OpenLoop)'); grid on; title('Yawwinkel','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    linkaxes(ax,'x');
 
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%% 3) Höhen-Vergleich (Baro,GPS,Kalman)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

   figure(2);
    set(gcf,'name',sprintf('3) Höhen-Vergleich (Baro,GPS,Kalman) (Datenreihe: %s)',Datenreihe),'numbertitle','off')
    plot(TIME_StartTime, [sim_GPOS_Alt,XGPS_Alt,XSEN_BaroAlt,XGPO_Alt,SENS_BaroAlt, GPOS_Alt]); grid on;
    legend('kalman Simuliert','XSENS GPS','XSENS Baro','XSENS Kalmann','PX4 Baro', 'PX4 Kalmann','PALT'); grid on;  title('Höhenvergleich (Baro, GPS, Kalman)','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Höhe [m]');
    
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%% 11) VelNED Vergleich (GPS, Kalman, Berechnet)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    dt_x = XIMU_Time(i+1)-XIMU_Time(i);
    
    xsens_VelN(i+1) = xsens_VelN(i) + xsens_AccMat_ned(1,(i+1))*dt_x;
    xsens_VelE(i+1) = xsens_VelE(i) + xsens_AccMat_ned(2,(i+1))*dt_x;
    xsens_VelD(i+1) = xsens_VelD(i) + xsens_AccMat_ned(3,(i+1))*dt_x;
    
    % PX4
    dt_p = IMU_Time(i+1)-IMU_Time(i);
   
    px4_VelN(i+1) = px4_VelN(i) + px4_AccMat_ned(1,(i+1))*dt_p;
    px4_VelE(i+1) = px4_VelE(i) + px4_AccMat_ned(2,(i+1))*dt_p;
    px4_VelD(i+1) = px4_VelD(i) + px4_AccMat_ned(3,(i+1))*dt_p; 
    
end
% nAn durch 0 ersetzen        
    GPOS_VelN(isnan(GPOS_VelN)) = 0 ;
    GPOS_VelE(isnan(GPOS_VelE)) = 0 ;
    GPOS_VelD(isnan(GPOS_VelD)) = 0 ;
 
 % Plot Down
    figure(3);
    set(gcf,'name',sprintf('11) VelNED Vergleich (GPS, Kalman, Berechnet) (Datenreihe: %s)',Datenreihe),'numbertitle','off')

    bx(1) = subplot(3,1,1)
    plot(XIMU_Time, [sim_GPOS_VelN GPOS_VelN, XGPO_VelN, XGPS_VelN]); hold on;
    legend('Kalman Simuliert','Px4 Kalman','Xsens Kalman','GPS'); grid on; title('North','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Geschw [m/s]');

    bx(2) = subplot(3,1,2);
    plot(XIMU_Time, [sim_GPOS_VelE GPOS_VelE, XGPO_VelE, XGPS_VelE]); hold on;
     legend('Kalman Simuliert','Px4 Kalman','Xsens Kalman','GPS'); grid on; title('East','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Geschw [m/s]');

    bx(3) = subplot(3,1,3);
    plot(XIMU_Time, [sim_GPOS_VelE GPOS_VelE, XGPO_VelE, XGPS_VelE]); hold on;
     legend('Kalman Simuliert','Px4 Kalman','Xsens Kalman','GPS'); grid on; title('Down','FontWeight','bold');
    xlabel('Zeitvektor [s]'); ylabel('Geschw [m/s]');
    linkaxes(bx,'x');
    