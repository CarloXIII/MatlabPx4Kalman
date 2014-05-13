function [ data ] = C_rpFromEuler( roll, pitch, yaw )
% *******************************************************%
% [ data ] = C_rpFromEuler( roll, pitch, yaw )           %
%  data:                                                 %
%                                                        %                  
% create a rotation matrix from given euler angles       %
% (function from mathlib/math/Matrix.hpp                 %
%  (from_euler)                                          %
%                                                        %
% Autor: Carlo Schuler                                   %
% Version: 1.0                                           %
% *******************************************************%
    	cp = cos(pitch);
		sp = sin(pitch);
		sr = sin(roll);
		cr = cos(roll);
		sy = sin(yaw);
		cy = cos(yaw);

		data(1,1) = cp * cy;
		data(1,2) = (sr * sp * cy) - (cr * sy);
		data(1,3) = (cr * sp * cy) + (sr * sy);
		data(2,1) = cp * sy;
		data(2,2) = (sr * sp * sy) + (cr * cy);
		data(2,3) = (cr * sp * sy) - (sr * cy);
		data(3,1) = -sp;
		data(3,2) = sr * cp;
		data(3,3) = cr * cp;

end

