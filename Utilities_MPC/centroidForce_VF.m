classdef centroidForce_VF< matlab.System
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
        LegStateOld=ones(4,1);
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
%             obj.ki_err_Old=obj.ki_err_Old+Ki*reshape(xRef(1:6)-xFB(1:6),6,1);
%             err=Kp*reshape(xRef(1:6)-xFB(1:6),6,1)+Kd*reshape(xRef(7:12)-xFB(7:12),6,1)+obj.ki_err_Old;
            obj.ki_err_Old=obj.ki_err_Old+Ki*[errpCoM;errSita];
            %err=Kp*reshape(xRef(1:6)-xFB(1:6),6,1)+Kd*reshape(xRef(7:12)-xFB(7:12),6,1)+obj.ki_err_Old;
            if LegState(1)+obj.LegStateOld(1)>0.8 && LegState(1)+obj.LegStateOld(1)<1.2
                obj.ki_err_Old=obj.ki_err_Old*0;
            end
            if LegState(2)+obj.LegStateOld(2)>0.8 && LegState(2)+obj.LegStateOld(2)<1.2
                obj.ki_err_Old=obj.ki_err_Old*0;
            end
            err=Kp*[errpCoM;errSita]+Kd*[errvCoM;errdSita]+obj.ki_err_Old;
            
            M=[LegState(1).*eye(3),LegState(2).*eye(3),LegState(3).*eye(3),LegState(4).*eye(3);...
                 LegState(1).*crossCap(Rz'*(pW(:,1)-pC)),LegState(2).*crossCap(Rz'*(pW(:,2)-pC)),LegState(3).*crossCap(Rz'*(pW(:,3)-pC)),LegState(4).*crossCap(Rz'*(pW(:,4)-pC));];
            
            Minv=pinv(M,10^-7);
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

%             M14=M(:,[1:3,10:12]);
%             M23=M(:,4:9);
%             if Disable>0.5
%                 U=zeros(12,1);
%                 obj.ki_err_Old=zeros(6,1);
%                 obj.sitaStore=zeros(3,400);
%                 obj.yawOld=zeros(1,10);
%             else
%                 if LegState(1)>0.5
%                     U14=pinv(M14,10^-7)*err;
%                 else
%                     U14=zeros(6,1);
%                 end
%                 if LegState(2)>0.5
%                     U23=pinv(M23,10^-7)*err;
%                 else
%                     U23=zeros(6,1);
%                 end
%                 U=[U14(1:3);U23;U14(4:6)];
%                 obj.sitaStore(:,1:end-1)=obj.sitaStore(:,2:end);
%                 obj.sitaStore(:,end)=xFB(4:6);
%             end


            for i=1:1:4
                U(3*i-2:3*i)=Rz*U(3*i-2:3*i);
            end
            
%             sitaAva=svd(M);
%             sitaStd=std(obj.sitaStore,0,2)/pi*180;
            sitaAva=0;
            sitaStd=0;
            obj.UOld=U;
            obj.LegStateOld=LegState;
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