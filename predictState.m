function [returnVal] = predictState(dt)
% *******************************************************%
% [returnVal] = predictState(dt)                         %
%  returnVal: 1 - if succesfull                          %
%                                                        %                  
% prediction step                                        %
% (function from att_pos_estimator/KalmanNav.cpp)        %
%  (KalmanNav::predictState(dt))                         %
%                                                        %
% Autor: Carlo Schuler                                   %
% Version: 1.0                                           %
% *******************************************************%
global  sensors_gyro_rad_s sensors_accelerometer_m_s2...
       attitudeInitialised positionInitialised...
       q C_nb phi theta psi R0 OMEGA  g...
    fN fE fD vN vE vD lat lon alt

sinL = sin(lat);
cosL = cos(lat);
cosLSing = cos(lat);

%prevent singularity
if abs(cosLSing) < 0.01
    if cosLSing > 0
        cosLSing = 0.01;
    else
        cosLSing = -0.01;
    end
end

% attitude prediction
%%%%%%%%%%%%%%%%%%%%%%
if attitudeInitialised
    w = sensors_gyro_rad_s;
    
    %attitude (Lagewinkel summiert)
    q_derivative = 0.5 * [-q(2) -q(3) -q(4);...
                           q(1) -q(4) q(3);...
                           q(4)  q(1) -q(2);...
                           -q(3) q(2) q(1)]*[w(1); w(2); w(3)];
    q = q + q_derivative' * dt;
    
    % renormalize quaternion if neede
    if abs(norm(q)-1)>(1e-4)
        quatnormalize(q);
    end
    
    % C_nb update
    C_nb = qToDCM(q);
    
    % euler update 
    euler = eulerFromDCM(C_nb);
    phi = euler(1);
    theta = euler(2);
    psi = euler(3);
    % specific acceleration in nav frame
    accelB = sensors_accelerometer_m_s2;
    accelN = C_nb * accelB;
    fN = accelN(1);
    fE = accelN(2);
    fD = accelN(3);
end

% Position Prediction
%%%%%%%%%%%%%%%%%%%%%

if positionInitialised
    R = R0 + alt;
    LDot = vN / R;
    lDot = vE / (cosLSing * R);
    rotRate = 2 * OMEGA + lDot;
    
    % XXX position prediction using speed
	vNDot = fN - vE * rotRate * sinL + vD * LDot;
	vDDot = fD - vE * rotRate * cosL - vN * LDot + g;
	vEDot = fE + vN * rotRate * sinL + vDDot * rotRate * cosL;

    % rectangular integration
	vN = vN + vNDot * dt;
	vE = vE + vEDot * dt;
	vD = vD + vDDot * dt;
	lat = lat + LDot * dt; %double(LDot*dt)
	lon = lon + lDot * dt; %double(lDot*dt)
	alt = alt + (-vD * dt); %double(-vD*dt)
end
returnVal = 1;
end

	
    
    
    
        

