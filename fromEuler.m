function [ data ] = fromEuler( roll,pitch,yaw )
% *******************************************************%
% [ data ] = fromEuler( roll,pitch,yaw )                 %
%  data: quaternions                                     %
%                                                        %                  
% set quaternion to rotation defined by euler angles     %
% (function from mathlib/math/Quaternion.hpp)            %
%  (from_euler)                                          %
%                                                        %
% Autor: Carlo Schuler                                   %
% Version: 1.0                                           %
% *******************************************************%
    	cosPhi_2 = cos(roll/2);
		sinPhi_2 = sin(roll/2);
		cosTheta_2 = cos(pitch/2);
		sinTheta_2 = sin(pitch/2);
		cosPsi_2 = cos(yaw/2.0);
		sinPsi_2 = sin(yaw/2.0);
		data(1) = cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2;
		data(2) = sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2;
		data(3) = cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2;
		data(4) = cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2;

end

