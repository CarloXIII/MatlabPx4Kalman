        % KalmanFilter Main-File

%% 1) Laden der Datenreihe

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
global TEST_NUMBER
TEST_NUMBER = 5
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2)  Daten einlesen
% Logging Frequenz 100 Hz
if(TEST_NUMBER == 6)
    load('Datenreihen\GyroTest');
    Datenreihe = 'GyroTest_sim'
    GPSENABLE = 0;
end
if(TEST_NUMBER == 5)
    load('Datenreihen\Flug3');
    Datenreihe = 'Flug3_sim'
    GPSENABLE = 1;
end
if(TEST_NUMBER == 4)
    load('Datenreihen\KalmanTestFeld');
    Datenreihe = 'KalmanTestFeld_sim'
    GPSENABLE = 1;
end
if(TEST_NUMBER == 3)
    load('Datenreihen\KalmanTestSpaz1');
    Datenreihe = 'Flug3_sim'
end
if(TEST_NUMBER == 2)
    load('Datenreihen\Flug2');
    Datenreihe = 'Flug2_sim'
    GPSENABLE = 1;
end
if(TEST_NUMBER == 1)
    load('Datenreihen\Flug1');
    Datenreihe = 'Flug1_sim'
    GPSENABLE = 1;
end

counterplot = 0;
%% 2) Deklaration Globale Variablen
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global  sensors_gyro_rad_s sensors_accelerometer_m_s2 sensors_magnetometer_ga sensors_baro_alt_meter 
global gps_lat gps_lon gps_alt gps_vel_n_m_s gps_vel_e_m_s gps_vel_d_m_s
global  attitudeInitialised positionInitialised
global  C_nb q F G P V R0 P0 HPos RPos HAtt RAtt
global phi theta psi fN fE fD vN vE vD lat lon alt
global  M_RAD_TO_DEG OMEGA g
global magDec faultPos faultAtt

attitudeInitialised = 0;
positionInitialised = 0;
attitudeInitCounter = 0;

% 3) Konstanten
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Titterton pg. 52
OMEGA = 7.2921150e-5; % earth rotation rate, rad/s
R0 = 6378137.0; % earth radius, m
G0 = 9.806; % standard gravitational accel. m/s^2
RET_OK = 0; 		% no error in function
RET_ERROR = -1; 	% error occurred
M_RAD_TO_DEG = (360/(2*pi));
% Veränderbare Parameter von Bodenstation
KF_V_GYRO = 0.008       %0.008
KF_V_ACCEL = 0.5        %1.0
KF_R_MAG = 0.8          %0.8
KF_R_GPS_VEL = 4      %0.5
KF_R_GPS_POS = 3      %2.0
KF_R_GPS_ALT = 30      %3.0
KF_R_PRESS_ALT = 1   %0.1    
KF_R_ACCEL = 0.4          %1.0
KF_ENV_MAG_DIP = 60.0   %60.0
KF_ENV_MAG_DEC = 0.0    %0.0
KF_ENV_G = 9.765        %9.765
KF_FAULT_POS = 10.0     %10.0
KF_FAULT_ATT = 10.0     %10.0

% Nur für Simulation
INITIALCOUNTER_LIMIT =100

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
%
%% 4) Initialisierung der Werte (z.54 in KalmanNav.cpp)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialwerte von Konstruktor (KalmanNav.cpp)
% Deklarationen
%--------------

% timestamps
pubTimeStamp = TIME_StartTime(1);
predictTimeStamp = TIME_StartTime(1);
attTimeStamp = TIME_StartTime(1);
outTimeStamp = TIME_StartTime(1);
% frame count
navFrames = 0;
% miss count
miss = 0;
% accelerations
fN = 0; fE = 0; fD = 0; 
%states
phi = 0; theta = 0; psi = 0; 
vN = 0; nE = 0; vD = 0;
lat = 0; lon = 0; alt = 0;
lat0 = 0; lon0 = 0; alt0 = 0;
% parameters from groundstation
vGyro = KF_V_GYRO;
vAccel = KF_V_ACCEL;
rMag = KF_R_MAG;
rGpsVel = KF_R_GPS_VEL;
rGpsPos = KF_R_GPS_POS;
rGpsAlt = KF_R_GPS_ALT;
rPressAlt = KF_R_PRESS_ALT;
rAccel = KF_R_ACCEL;
magDip = KF_ENV_MAG_DIP;
magDec = KF_ENV_MAG_DEC;
g = KF_ENV_G;
faultPos = KF_FAULT_POS;
faultAtt = KF_FAULT_ATT;


% (KalmanNav.cpp (z.100))
% Init
F = zeros(9,9); % Jacobian(f,x), where dx/dt = f(x,u)
G = zeros(9,6); % noise shaping matrix for gyro/accel
V = zeros(6,6); % gyro/ accel noise matrix
HAtt = zeros(4,9); % attitude measurement matrix
RAtt = zeros(4,4); % attitude measurement noise matrix
HPos = zeros(6,9); % position measurement jacobian matrix
RPos = zeros(6,6); % position measurement noise matrix

% initial state covariance matrix
P0 = eye(9,9); % initial state covariance matrix (diagonalmatrix 9,9)
P0 = P0 * 0.01;
P = P0;

% initial state
phi = 0.0;
theta = 0.0;
psi = 0.0;
vN = 0.0;
vE = 0.0;
vD = 0.0;
lat = 0.0;
lon = 0.0;
alt = 0.0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OUTPUTS Initialisierungen (nur für Simulation in Matlab)
% global position
sim_GPOS_Time = zeros(length(TIME_StartTime),1);
sim_GPOS_Lat = zeros(length(TIME_StartTime),1);
sim_GPOS_Lon = zeros(length(TIME_StartTime),1);
sim_GPOS_Alt = zeros(length(TIME_StartTime),1);
sim_GPOS_VelN = zeros(length(TIME_StartTime),1);
sim_GPOS_VelE = zeros(length(TIME_StartTime),1);
sim_GPOS_VelD = zeros(length(TIME_StartTime),1);
sim_GPOS_yaw = zeros(length(TIME_StartTime),1);

% attitude
sim_ATT_Time = zeros(length(TIME_StartTime),1);
sim_ATT_Roll = zeros(length(TIME_StartTime),1);
sim_ATT_Pitch = zeros(length(TIME_StartTime),1);
sim_ATT_Yaw = zeros(length(TIME_StartTime),1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% INIT KALMAN

% Aktuelle Messung
in = 1;
% INIT KALMAN

q = init(XIMU_AccX(in),...
         XIMU_AccY(in),...
         XIMU_AccZ(in),...
         XIMU_MagX(in),...
         XIMU_MagY(in),...
         XIMU_MagZ(in));
% initialise dcm
C_nb = qToDCM(q);

% HPos is constant
HPos(1,4) = 1.0;
HPos(2,5) = 1.0;
HPos(3,7) = (1e7)*M_RAD_TO_DEG;
HPos(4,8) = (1e7)*M_RAD_TO_DEG;
HPos(5,9) = 1.0;
HPos(6,9) = 1.0;
     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
%
%% updateParams Methode (z.755)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% gyro noise
V(1,1) = vGyro;
V(2,2) = vGyro;
V(3,3) = vGyro;

% accel noise
V(4,4) = vAccel;
V(5,5) = vAccel;
V(6,6) = vAccel;


noiseMin = 1e-6;
% magnetometer noise
noiseMagSq = rMag^2;
if (noiseMagSq < noiseMin)
    noiseMagSq = noiseMin;
end
RAtt(1,1) = noiseMagSq; % normalized direction

% accelerometer noise
noiseAccelSq = rAccel^2;
if(noiseAccelSq < noiseMin)
    noiseAcceSq = noiseMin;
end
RAtt(2,2) = noiseAccelSq; %normalized direction
RAtt(3,3) = noiseAccelSq;
RAtt(4,4) = noiseAccelSq;

% gps noise
R = R0 + alt;
cosLSing = cos(lat);

% prevent singularity
if(abs(cosLSing) < 0.01)
    if(cosLSing > 0)
        cosLSing = 0.01;
    else
        cosLSing = -0.01;
    end
end

noiseVel = rGpsVel;
noiseLatDegE7 = (1e7)*M_RAD_TO_DEG * rGpsPos/R;
noiseLonDegE7 = noiseLatDegE7 / cosLSing;
noiseGpsAlt = rGpsAlt;
noisePressAlt = rPressAlt;

% bound noise to prevent singularities
if (noiseVel < noiseMin) 
    noiseVel = noiseMin;
end

if (noiseLatDegE7 < noiseMin) 
    noiseLatDegE7 = noiseMin;
end

if (noiseLonDegE7 < noiseMin) 
    noiseLonDegE7 = noiseMin;
end

if (noiseGpsAlt < noiseMin) 
    noiseGpsAlt = noiseMin;
end

if (noisePressAlt < noiseMin) 
    noisePressAlt = noiseMin;
end

RPos(1, 1) = noiseVel^2; % vn
RPos(2, 2) = noiseVel^2; % ve
RPos(3, 3) = noiseLatDegE7^2; % lat
RPos(4, 4) = noiseLonDegE7^2; % lon
RPos(5, 5) = noiseGpsAlt^2; % h
RPos(6, 6) = noisePressAlt^2; % h
%%%%%%%%%%%%%%%%%%%%      END update Params        %%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%     END KalmanNav.cpp Konstruktor       %%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                    %
                                    %
                                    %
                                    %
                                    %
                                    %
                                    %
                                    %
                                    %
                                    %
                                    %
                                    %
                                    %
                                    %
                                    %
                                    %
                                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%










for MSTind=2:1:(length(TIME_StartTime)-1)/1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
%
%%                       UPDATE - METHODE (z.191)
%                        ------------------------
% 
% Diese Methode wird von der Main-Methode des att_pos_estimators ständig 
% aufgerufen
%
%
% Definiert das eigentliche Verhalten des Kalman Filters
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%getNewTimestamp
newTimeStamp = TIME_StartTime(MSTind); % us

%todo Carlo Test für Timestamps (FAKE_TIME)
%newTimeStamp = TIME_StartTime(MSTind)+(0.01/SUBSTEPS);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 	updateSubscriptions();
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sensors_accelerometer_m_s2 = [XIMU_AccX(MSTind);...
                              XIMU_AccY(MSTind);...
                              XIMU_AccZ(MSTind)];
sensors_gyro_rad_s = [XIMU_GyroX(MSTind);...
                      XIMU_GyroY(MSTind);...
                      XIMU_GyroZ(MSTind)];
sensors_magnetometer_ga = [XIMU_MagX(MSTind);...
                           XIMU_MagY(MSTind);...
                           XIMU_MagZ(MSTind)];
sensors_baro_alt_meter = XSEN_BaroAlt(MSTind);
sensors_timestamp = XIMU_Time(MSTind);
gps_vel_n_m_s = XGPS_VelN(MSTind);
gps_vel_e_m_s = XGPS_VelE(MSTind);
gps_vel_d_m_s = XGPS_VelD(MSTind);
gps_lat = XGPS_Lat(MSTind);
gps_lon = XGPS_Lon(MSTind);
gps_alt = XGPS_Alt(MSTind);

% Überprüfen das GPS nicht gleiche werte als neue deklariert
if MSTind  ~= 1
    if sensors_accelerometer_m_s2(1) ~= XIMU_AccX(MSTind - 1)
        sensorUpdate = 1; % TEST für Sensorupdate
    end
    if gps_lat ~= XGPS_Lat(MSTind - 1);
        gpsUpdate = 1;
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 	Initialize attitude when sensors online
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if attitudeInitialised == 0 && sensorUpdate
		if (correctAtt() == 1) 
            attitudeInitCounter = attitudeInitCounter + 1;
        end
		if (attitudeInitCounter > INITIALCOUNTER_LIMIT) 
			disp([sprintf('Attitude initialised: Roll: %d, Pitch: %d, Yaw: %d',...
            phi,theta,psi) char(10)]);
			attitudeInitialised = 1;
        end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Initialize position when gps received
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if  (positionInitialised == 0) && (attitudeInitialised == 1) && gpsUpdate && GPSENABLE% && gps.fix_typ > 2
 		vN = gps_vel_n_m_s;
        vE = gps_vel_e_m_s;
        vD = gps_vel_d_m_s;
        lat = gps_lat / 1e7 / M_RAD_TO_DEG;
        lon = gps_lon / 1e7 / M_RAD_TO_DEG;
        alt = gps_alt / 1e3;
        
%		set reference position for
%		local position
		lat0 = lat;
		lon0 = lon;
		alt0 = alt;
        
 		positionInitialised = 1;
        disp(['Position init with GPS', char(10)]);
        disp([sprintf('vN: %d, vE: %d, vD: %d, lat: %d, lon: %d, alt: %d',...
            vN,vE,vD,lat,lon,alt) char(10)]);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PREDICTION STEP (KalmanNav.cpp Z.271)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt = (XIMU_Time(MSTind)-XIMU_Time(MSTind-1))/1e6;
predictTimeStamp = sensors_timestamp;

if(dt < 1.0)
    predictState(dt);
    predictStateCovariance(dt);
    navFrames = navFrames + 1;
end
% count times 100Hz isnt met
if(dt > 0.1)
    miss = miss + 1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GPS correction step
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if positionInitialised && gpsUpdate
    correctPos();
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Attitude correction step
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if attitudeInitialised && sensorUpdate %&& (sensors_timestamp - attTimeStamp > 1e6/50) %50 Hz
       attTimeStamp = sensors_timestamp;
       correctAtt();
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Publications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%if newTimeStamp -pubTimeStamp > 1e6 / 50

   pubTimeStamp = sensors_timestamp;
   
   
    % Global Position
    sim_GPOS_Time(MSTind) = pubTimeStamp;
    sim_GPOS_Lat(MSTind) = lat * M_RAD_TO_DEG;
    sim_GPOS_Lon(MSTind) = lon * M_RAD_TO_DEG;
    sim_GPOS_Alt(MSTind) = alt;
    sim_GPOS_VelN(MSTind) = vN;
    sim_GPOS_VelE(MSTind) = vE;
    sim_GPOS_VelD(MSTind) = vD;
    sim_GPOS_yaw(MSTind) = psi;
    
    % Attitude
    sim_ATT_Time(MSTind) = pubTimeStamp;
    sim_ATT_Roll(MSTind) = phi;
    sim_ATT_Pitch(MSTind) = theta;
    sim_ATT_Yaw(MSTind) =psi;
    
%end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output (not used in C-Code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if newTimeStamp -outTimeStamp > 10e6 %0.1 Hz
    outTimeStamp = newTimeStamp;
    navFrames = 0;
    miss = 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%todo Carlo 
%gpsUpdate = 0; 
sensorUpdate = 0;
end

%% Speichern des Outputs für plots
saveName = strcat('Datenreihen\',Datenreihe)
save(saveName);

     
     

     
     
















     
     
        