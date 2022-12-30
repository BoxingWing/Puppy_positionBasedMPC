classdef TrustRegion < matlab.System
    properties
        k=10;
        Cphi_W=0.1;
        Cz_kp=100;
        Cz_kn=20;
    end

    properties (Access=private)

    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants

        end

        function [Xi,diagXi] = stepImpl(obj,LegState,LegPhi,pW)
           pW_M=reshape(pW,3,4);
           C=zeros(3,3,4);
           for i=1:1:4
                Cphi=CphiFun(LegPhi(i),LegState(i),obj.Cphi_W);
                Cz=CzFun(pW_M(3,i),obj.Cz_kp,obj.Cz_kn);
                C(:,:,i)=Cphi*diag([1,1,Cz]);
           end
           Xi=eye(12)+obj.k*(eye(12)-blkdiag(C(:,:,1), ...
               C(:,:,2),C(:,:,3),C(:,:,4)));
           diagXi=diag(Xi);
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

    end
end
function y=CphiFun(phi,s,W)
    y=s*(erf(12*phi/W-6)+erf(12*(1-phi)/W-6)-1);
end

function y=CzFun(pz,kp,kn)
% kp=100;
% kn=20;
if pz>=0
    y=exp(-kp*pz^2);
else
    y=exp(-kn*pz^2);
end
end

function y=erf(x)
    y=1/(1+exp(-x));
end