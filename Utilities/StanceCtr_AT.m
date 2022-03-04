classdef StanceCtr_AT< matlab.System
    % stance leg control module
    properties
        tSW=0.3;
        tSample=0.005;
        r0=0.19;
        lateral_width=0.097;
        sagetial_width=0.2108;
        roll_Off=0.037;
        kp=zeros(1,6);
        kd=zeros(1,6);
        ki=zeros(1,6);
    end

    properties(Access=private)
        isLFnext=true;
        isRFnext=false;
        swCount_LF=0;
        swCount_RF=0;
        swN=1;
        yawOld=zeros(1,10);
        LegStateOld=[1;1;1;1];
        vCoM_sw=[1;1;1]; % CoM velocity at the beginning of the last swing phase
        ki_err_Old=zeros(6,1);
    end

    methods(Access = protected)

        function setupImpl(obj)
            obj.swN=floor(obj.tSW/obj.tSample);
            obj.isLFnext=true;
            obj.isRFnext=false;
            obj.swCount_LF=0;
            obj.swCount_RF=0;
            obj.swN=1;
            obj.LegStateOld=[1;1;1;1];
            obj.vCoM_sw=[1;1;1];
        end

        function [pL_st,fL] = stepImpl(obj,pL_old,pB_old,pW_m,xFB,xRef,LegState,Disable)
            % pL_old: [3,4] baseline pL in the last step
            % pB_old: [3,4] baseline pB in the last step
            % pW_m : measured pW
            % fL: foot end contact forces in the leg coordinate
            % xDes: [vx,vy,wz], in the local body frame

            % norminal foot end positions
            pL_old=reshape(pL_old,3,4);
            pB_old=reshape(pB_old,3,4);
            xRef=-xRef;
            if Disable>0.5
                pL_st=reshape([0;obj.roll_Off;-obj.r0;0;-obj.roll_Off;-obj.r0; ...
                    0;obj.roll_Off;-obj.r0;0;-obj.roll_Off;-obj.r0;],3,4);
            else
                
%                 if xRef(7)<10^-3 && xRef(8)<10^-3 % to clear the foot position when Vdef=0
%                     
%                 else
%                     linAct=[xRef(7);xRef(8);0]*[1,1,1,1]*obj.tSample;
%                 end
                linAct=[xRef(7);xRef(8);0]*[1,1,1,1]*obj.tSample;
                pL_st=pL_old+linAct+...
                    [cross(pB_old(:,1),[0;0;xRef(12)]),cross(pB_old(:,2),[0;0;xRef(12)]), ...
                    cross(pB_old(:,3),[0;0;xRef(12)]),cross(pB_old(:,4),[0;0;xRef(12)])]*obj.tSample;
            end

            % contact forces control
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
                LegState(1).*crossCap(Rz'*(pW_m(:,1)-pC)),LegState(2).*crossCap(Rz'*(pW_m(:,2)-pC)),LegState(3).*crossCap(Rz'*(pW_m(:,3)-pC)),LegState(4).*crossCap(Rz'*(pW_m(:,4)-pC));];

            Minv=pinv(M,10^-7);
            if Disable>0.5
                U=zeros(12,1);
                obj.ki_err_Old=zeros(6,1);
                obj.yawOld=zeros(1,10);
            else
                U=Minv*err;
            end
%             for i=1:1:4
%                 U(3*i-2:3*i)=Rz*U(3*i-2:3*i);
%             end
            fL=U;

            obj.LegStateOld=LegState;

            pL_st=reshape(pL_st,12,1);
            fL=reshape(fL,12,1);
        end
        
        function [d1,d2] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'double';
        end

        function [s1,s2] = getOutputSizeImpl(~)
            s1 = [12,1];
            s2=[12,1];
        end

        function [f1,f2] = isOutputFixedSizeImpl(~)
            f1 = true;
            f2 = true;
        end

        function [cpl1,cpl2] = isOutputComplexImpl(~)
            cpl1 = false;
            cpl2=false;
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


