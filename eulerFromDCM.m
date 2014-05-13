function [euler] = eulerFromDCM( data )
% *******************************************************%
% [euler] = eulerFromDCM( data )                         %
%  euler:                                                %
%                                                        %                  
% get euler angles from rotation matrix                  %
% (function from mathlib/math/Matrix.hpp)                %
%  (to_euler)                                            %
%                                                        %
% Autor: Carlo Schuler                                   %
% Version: 1.0                                           %
% *******************************************************%
		euler(2) = asin(-data(3,1));

		if (abs(euler(2)-(pi/2)) < (1e-3)) 
			euler(1) = 0;
			euler(3) = atan2(data(2,3) - data(1,2), data(1,3) + data(2,2)) + euler(1);
        elseif (abs(euler(2) + (pi/2)) < (1e-3)) 
			 euler(1) = 0;
			 euler(3) = atan2(data(2,3) - data(1,2), data(1,3) + data(2,2)) - euler(1);
        else
            euler(1) = atan2(data(3,2), data(3,3));
            euler(3) = atan2(data(2,1), data(1,1));
        end
end

