function [ returnVal ] = correctPos()
% *******************************************************%
% [ returnVal ] = correctPos()                           %
%  returnVal: 1 - if succesfull                          %
%                                                        %                  
% Position correction step                               %
% (function from att_pos_estimator/KalmanNav.cpp)        %
%  (KalmanNav::correctPos())                             %
%                                                        %
% Autor: Carlo Schuler                                   %
% Version: 1.0                                           %
% *******************************************************%
    global gps_vel_n_m_s gps_vel_e_m_s gps_vel_d_m_s gps_lat gps_lon gps_alt sensors_baro_alt_meter
    global vN vE vD lat lon alt
    global HPos P RPos P0
    global M_RAD_TO_DEG faultPos
    
    % residual
    y = [gps_vel_n_m_s - vN;...
         gps_vel_e_m_s - vE;...
         gps_lat - lat * 1e7 * M_RAD_TO_DEG;...
         gps_lat - lon * 1e7 * M_RAD_TO_DEG;...
         (gps_alt/1e3) - alt;...
         sensors_baro_alt_meter - alt];

     % compute correction
     S = HPos * P * transpose(HPos) + RPos;
     K = P * transpose(HPos)*inv(S);
     xCorrect = K * y;
     
     % check correction is sane
for i=1:1:size(xCorrect,2)
    for l=1:1:size(xCorrect,1)
        val = xCorrect(l,i);
        if isinf(val)
            % abort correction and return
            disp(['Numerical failure in gps correction', char(10)]);
            % fallback to gps
            vN = gps_vel_n_m_s;
            vE = gps_vel_e_m_s;
            vD = gps_vel_d_m_s;
            lat = gps_lat / 1e7 / M_RAD_TO_DEG;
            lon = gps_lon / 1e7 / M_RAD_TO_DEG;
            alt = gps_alt / 1e3;
            P=P0;
            returnVal = 0;
        end
    end
end

    % correct state
    vN = vN + xCorrect(4);
    vE = vE + xCorrect(5);
    vD = vD + xCorrect(6);
    lat = lat + double(xCorrect(7));
    lon = lon + double(xCorrect(8));
    alt = alt + xCorrect(9);
    
    % update state covariance
    % http://en.wikipedia.org/wiki/Extended_Kalman_filter
    P = P -K*HPos*P;
    
    % fault detection
    beta = y'*(inv(S)*y);
    
    persistent counter
    if (beta > faultPos)%&&(counter == 10))
        %disp(['fault in GPS', char(10)]);
    end
    if counter == 10
        counter = 0;
    else
        counter = counter + 1;
    end
    
    returnVal = 1;

end

