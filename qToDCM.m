function [ dcm ] = qToDCM( data)
% *******************************************************%
% function [dcm] = qToDCM( data)                         %
% data: quaternions (p1 p2 p3 p4)                       %
% dcm:  rotation-matrix                                  %
%                                                        %
% Transformation from quaternions to                     %
% rotation-matrix                                        %
%                                                        %
% Autor: Carlo Schuler                                   %
% Version: 1.0                                           %
% *******************************************************%
		aSq = data(1) * data(1);
		bSq = data(2) * data(2);
		cSq = data(3) * data(3);
		dSq = data(4) * data(4);
		dcm(1,1) = aSq + bSq - cSq - dSq;
		dcm(1,2) = 2.0 * (data(2) * data(3) - data(1) * data(4));
		dcm(1,3) = 2.0 * (data(1) * data(3) + data(2) * data(4));
		dcm(2,1) = 2.0 * (data(2) * data(3) + data(1) * data(4));
		dcm(2,2) = aSq - bSq + cSq - dSq;
		dcm(2,3) = 2.0 * (data(3) * data(4) - data(1) * data(2));
		dcm(3,1) = 2.0 * (data(2) * data(4) - data(1) * data(3));
		dcm(3,2) = 2.0 * (data(1) * data(2) + data(3) * data(4));
		dcm(3,3) = aSq - bSq - cSq + dSq;
		

end

