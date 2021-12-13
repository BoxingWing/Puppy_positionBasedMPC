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
        yawOld=zeros(1,10);
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end
        
        function [U,sitaAva,sitaStd] = stepImpl(obj,LegState,pW,touchInd,xRef,xFB,Disable)
            pC=xFB(1:3);
            sita=xFB(4:6);

            obj.yawOld(1:end-1)=obj.yawOld(2:end);
            obj.yawOld(end)=xFB(6);
            yawFilt=sum(obj.yawOld)/length(obj.yawOld);
            Rz=[cos(yawFilt),-sin(yawFilt),0;sin(yawFilt),cos(yawFilt),0;0,0,1];
            
            %Rz=eye(3);

            T=[cos(sita(2))*cos(sita(3)),-sin(sita(3)),0;
                cos(sita(2))*sin(sita(3)),cos(sita(3)),0;
                -sin(sita(2)),0,1];
            %dsita=T\[xFB(10);xFB(11);xFB(12)];
            errSita=([xRef(4);xRef(5);xRef(6)]-[xFB(4);xFB(5);xFB(6)]);
            errdSita=T\([xRef(10);xRef(11);xRef(12)]-[xFB(10);xFB(11);xFB(12)]);
            errpCoM=Rz'*([xRef(1);xRef(2);xRef(3)]-[xFB(1);xFB(2);xFB(3)]);
            errvCoM=Rz'*([xRef(7);xRef(8);xRef(9)]-[xFB(7);xFB(8);xFB(9)]);

            Kp=diag(obj.kp);
            Kd=diag(obj.kd);
            Ki=diag(obj.ki);
            obj.ki_err_Old=obj.ki_err_Old+Ki*[errpCoM;errSita];
            %err=Kp*reshape(xRef(1:6)-xFB(1:6),6,1)+Kd*reshape(xRef(7:12)-xFB(7:12),6,1)+obj.ki_err_Old;
            err=Kp*[errpCoM;errSita]+Kd*[errvCoM;errdSita]+obj.ki_err_Old;
            
            M=[LegState(1).*eye(3),LegState(2).*eye(3),LegState(3).*eye(3),LegState(4).*eye(3);...
                 LegState(1).*crossCap(Rz'*(pW(:,1)-pC)),LegState(2).*crossCap(Rz'*(pW(:,2)-pC)),LegState(3).*crossCap(Rz'*(pW(:,3)-pC)),LegState(4).*crossCap(Rz'*(pW(:,4)-pC));];
            Minv=pinv(M);
            
            if Disable>0.5
                U=zeros(12,1);
                obj.ki_err_Old=zeros(6,1);
                obj.sitaStore=zeros(3,400);
                obj.yawOld=zeros(1,10);
            else
                U=Minv*err;
                obj.sitaStore(:,1:end-1)=obj.sitaStore(:,2:end);
                obj.sitaStore(:,end)=xFB(4:6);
            end

            for i=1:1:4
                U(3*i-2:3*i)=Rz*U(3*i-2:3*i);
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


