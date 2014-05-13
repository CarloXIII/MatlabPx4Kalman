function [returnVal] = predictStateCovariance(dt)
% *******************************************************%
% [returnVal] = predictStateCovariance(dt)               %
%  returnVal: 1 - if succesfull                          %
%                                                        %                  
% predict state covariance step                          %
% (function from att_pos_estimator/KalmanNav.cpp)        %
%  (KalmanNav::predictStateCovariance(dt))               %
%                                                        %
% Autor: Carlo Schuler                                   %
% Version: 1.0                                           %
% *******************************************************%
global vN vE vD fN fE fD lat alt
global R0 OMEGA
global F G P V C_nb


%trig
sinL = sin(lat);
cosL = cos(lat);
cosLSq = cosL^2;
tanL = tan(lat);

% prepare for matrix
R = R0 + alt;
RSq = R^2;
% F Matrix
% Titterton pg.291
    F(1, 2) = -(OMEGA * sinL + vE * tanL / R);
	F(1, 3) = vN / R;
	F(1, 5) = 1.0 / R;
	F(1, 7) = -OMEGA * sinL;
	F(1, 9) = -vE / RSq;

	F(2, 1) = OMEGA * sinL + vE * tanL / R;
	F(2, 3) = OMEGA * cosL + vE / R;
	F(2, 4) = -1.0 / R;
	F(2, 9) = vN / RSq;

	F(3, 1) = -vN / R;
	F(3, 2) = -OMEGA * cosL - vE / R;
	F(3, 5) = -tanL / R;
	F(3, 7) = -OMEGA * cosL - vE / (R * cosLSq);
	F(3, 9) = vE * tanL / RSq;

	F(4, 2) = -fD;
	F(4, 3) = fE;
	F(4, 4) = vD / R;
	F(4, 5) = -2 * (OMEGA * sinL + vE * tanL / R);
	F(4, 6) = vN / R;
	F(4, 7) = -vE * (2 * OMEGA * cosL + vE / (R * cosLSq));
	F(4, 9) = (vE * vE * tanL - vN * vD) / RSq;

	F(5, 1) = fD;
	F(5, 3) = -fN;
	F(5, 4) = 2 * OMEGA * sinL + vE * tanL / R;
	F(5, 5) = (vN * tanL + vD) / R;
	F(5, 6) = 2 * OMEGA * cosL + vE / R;
	F(5, 7) = 2 * OMEGA * (vN * cosL - vD * sinL) + vN * vE / (R * cosLSq);
	F(5, 9) = -vE * (vN * tanL + vD) / RSq;

	F(6, 1) = -fE;
	F(6, 2) = fN;
	F(6, 4) = -2 * vN / R;
	F(6, 5) = -2 * (OMEGA * cosL + vE / R);
	F(6, 7) = 2 * OMEGA * vE * sinL;
	F(6, 9) = (vN * vN + vE * vE) / RSq;

	F(7, 4) = 1 / R;
	F(7, 9) = -vN / RSq;

	F(8, 5) = 1 / (R * cosL);
	F(8, 7) = vE * tanL / (R * cosL);
	F(8, 9) = -vE / (cosL * RSq);

	F(9, 6) = -1;
    
    % G Matrix
    % Titterton pg. 291
    G(1, 1) = -C_nb(1, 1);
	G(1, 2) = -C_nb(1, 2);
	G(1, 3) = -C_nb(1, 3);
	G(2, 1) = -C_nb(2, 1);
	G(2, 2) = -C_nb(2, 2);
	G(2, 3) = -C_nb(2, 3);
	G(3, 1) = -C_nb(3, 1);
	G(3, 2) = -C_nb(3, 2);
	G(3, 3) = -C_nb(3, 3);

	G(4, 4) = C_nb(1, 1);
	G(4, 5) = C_nb(1, 2);
	G(4, 6) = C_nb(1, 3);
	G(5, 4) = C_nb(2, 1);
	G(5, 5) = C_nb(2, 2);
	G(5, 6) = C_nb(2, 3);
	G(6, 4) = C_nb(3, 1);
	G(6, 5) = C_nb(3, 2);
	G(6, 6) = C_nb(3, 3);

   % continuous prediction equations
	% for discrete time EKF
	% http://en.wikipedia.org/wiki/Extended_Kalman_filter
	P = P + (F * P + P * transpose(F) + G * V * transpose(G)) * dt;
    returnVal = 1;
    
end

