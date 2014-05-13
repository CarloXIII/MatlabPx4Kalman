function [ returnVal ] = correctAtt()
% *******************************************************%
% [ returnVal ] = correctAtt()                           %
%  returnVal: 1 - if succesfull                          %
%                                                        %                  
% Attitude correction step                               %
% (function from att_pos_estimator/KalmanNav.cpp)        %
%  (KalmanNav::correctAtt())                             %
%                                                        %
% Autor: Carlo Schuler                                   %
% Version: 1.0                                           %
% *******************************************************%
global phi theta psi vN vE vD
global magDec M_RAD_TO_DEG g
global sensors_magnetometer_ga sensors_accelerometer_m_s2
global RAtt HAtt C_nb P P0 q
global positionInitialised faultAtt
%trig
cosPhi = cos(phi);
cosTheta = cos(theta);
   % float cosPsi = cosf(psi);
sinPhi = sin(phi);
sinTheta = sin(theta);
   % float sinPsi = sinf(psi);

% mag predicted measurement
dec = magDec/M_RAD_TO_DEG;

% compensate roll and pitch, but not yaw         
C_rp = C_rpFromEuler(phi, theta, 0);

% mag measurement
magBody = sensors_magnetometer_ga;

% transform to earth frame                      
magNav = C_rp * magBody;

% calculate error between estimate and measurement
% apply declination correction for true heading as well
yMag = -atan2(magNav(2),magNav(1))-psi-dec;
if yMag > pi
    yMag = yMag - 2*pi;
end
if yMag < -pi
    yMag = yMag + 2*pi;
end

% Accel measurement
zAccel = sensors_accelerometer_m_s2;
accelMag = norm(zAccel); % length of vector
zAccel = zAccel/accelMag; % normalize vector

% ignore accel correction when accel mag not close to g
RAttAdjust = RAtt;
if abs(accelMag -g)> 1.1
    ignoreAccel = 1;
else
    ignoreAccel = 0;
end

if ignoreAccel == 1
    RAttAdjust(2,2) = 1e10;
    RAttAdjust(3,3) = 1e10;
    RAttAdjust(4,4) = 1e10;
end
    

% Accel predicted measurement
temp = C_nb'*[0;0;-g];
zAccelHat = temp/(norm(temp));
% Calculate residual
y = [yMag;zAccel(1)-zAccelHat(1);zAccel(2)-zAccelHat(2);zAccel(3)-zAccelHat(3)];

% HMag
HAtt(1,3) = 1;

% HAccel
HAtt(2, 2) = cosTheta;
HAtt(3, 1) = -cosPhi * cosTheta;
HAtt(3, 2) = sinPhi * sinTheta;
HAtt(4, 1) = sinPhi * cosTheta;
HAtt(4, 2) = cosPhi * sinTheta;

% compute correction
% http://en.wikipedia.org/wiki/Extended_Kalman_filter
S = HAtt * P * HAtt' + RAttAdjust; % residual covariance
K = P * HAtt'* inv(S);
xCorrect = K * y;

% check correction is sane
for i=1:1:size(xCorrect,2)
    for l=1:1:size(xCorrect,1)
        val = xCorrect(l,i);
        if isnan(val)
            P=P0;
            disp([sprintf('correction insane:') char(10)]);
            returnVal = 0;
        end
        if isinf(val)
            P=P0;
            disp([sprintf('correction insane:') char(10)]);
            returnVal = 0;
        end
    end
end

% correct state
if ignoreAccel == 0
    phi = phi + xCorrect(1);
    theta = theta + xCorrect(2);
end
psi = psi + xCorrect(3);

% attitude also affects nav velosities
if positionInitialised 
    vN = vN + xCorrect(4);
    vE = vE + xCorrect(5);
    vD = vD + xCorrect(6);
end

% update state covariance
% http://en.wikipedia.org/wiki/Extended_Kalman_filter
P = P - K * HAtt * P;

% fault detection 
beta = y' * (inv(S) * y);
if beta > faultAtt
   disp([sprintf('fault in attitude: beta %d:', beta) char(10)]);
end

% update quaternions from euler
% angle correction
q = fromEuler(phi, theta, psi);

returnVal = 1;

end

