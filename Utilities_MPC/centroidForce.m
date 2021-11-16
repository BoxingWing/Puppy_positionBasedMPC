classdef centroidForce < matlab.System
    % force and moment compensation via PD laws
    properties
        kp=zeros(1,6);
        kd=zeros(1,6);
        ki=zeros(1,6);
    end
    properties (Access=private)
        UOld=zeros(12,1);
        ki_err_Old=zeros(6,1);
        sitaStore=zeros(3,400);
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end
        
        function [U,sitaAva,sitaStd] = stepImpl(obj,LegState,pW,touchInd,xRef,xFB,Disable)
            pC=xFB(1:3);
            
            Kp=diag(obj.kp);
            Kd=diag(obj.kd);
            Ki=diag(obj.ki);
            obj.ki_err_Old=obj.ki_err_Old+Ki*reshape(xRef(1:6)-xFB(1:6),6,1);
            err=Kp*reshape(xRef(1:6)-xFB(1:6),6,1)+Kd*reshape(xRef(7:12)-xFB(7:12),6,1)+obj.ki_err_Old;
            M=[LegState(1).*eye(3),LegState(2).*eye(3),LegState(3).*eye(3),LegState(4).*eye(3);...
                 LegState(1).*crossCap(pW(:,1)-pC),LegState(2).*crossCap(pW(:,2)-pC),LegState(3).*crossCap(pW(:,3)-pC),LegState(4).*crossCap(pW(:,4)-pC);];
            Minv=pinv(M);
            
            if Disable>0.5
                U=zeros(12,1);
                obj.ki_err_Old=zeros(6,1);
                obj.sitaStore=zeros(3,400);
            else
                U=Minv*err;
                obj.sitaStore(:,1:end-1)=obj.sitaStore(:,2:end);
                obj.sitaStore(:,end)=xFB(4:6);
            end
            sitaAva=mean(obj.sitaStore,2)/pi*180;
            sitaStd=std(obj.sitaStore,0,2)/pi*180;
            obj.UOld=U;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end

function vcap=crossCap(v)
vcap=[0,-v(3),v(2);
        v(3),0,-v(1);
       -v(2),v(1),0];
end


