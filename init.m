function [ Quaternion ] = init(ax,ay,az,mx,my,mz)
% *******************************************************%
% function [Quaternion] = init(ax,ay,az,mx,my,mz)        %
% Quaternion: quaternions (p1 p2 p3 p4)                  %
% ax,ay,az:   accelerometer xyz                          %
% mx,my,mz   magnetometer xyz                            %
%                                                        %                  
% initialisation of the quaternions                      %
% (function from att_pos_estimator/KalmanNav.cpp)        %
%                                                        %
% Autor: Carlo Schuler                                   %
% Version: 1.0                                           %
% *******************************************************%
    
    initialRoll = atan2(-ay, -az);
    initialPitch = atan2(ax, -az);

    cosRoll = cos(initialRoll);
    sinRoll = sin(initialRoll);
    cosPitch = cos(initialPitch);
    sinPitch = sin(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2(-magY, magX);
% calculate the quaternions
    cosRoll = cos(initialRoll * 0.5);
    sinRoll = sin(initialRoll * 0.5);

    cosPitch = cos(initialPitch * 0.5);
    sinPitch = sin(initialPitch * 0.5);

    cosHeading = cos(initialHdg * 0.5);
    sinHeading = sin(initialHdg * 0.5);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

    Quaternion = [q0 q1 q2 q3];

end

