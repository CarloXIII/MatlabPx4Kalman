function [] = updateParams(  )
% updateParams Methode (z.755)

global V RAtt RPos M_RAD_TO_DEG
global vGyro vAccel rMag rAccel alt lat rGpsAlt rGpsVel rGpsPos rPressAlt

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


end

