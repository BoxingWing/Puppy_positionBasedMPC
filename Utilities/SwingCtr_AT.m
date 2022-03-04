classdef SwingCtr_AT< matlab.System
    % stance leg control module
    properties
        tSW=0.3;
        tSample=0.005;
        r0=[0;0;0.19];
        StepH=0.05;
        lateral_width=0.097;
        sagetial_width=0.2108;
        roll_Off=0.037;
        kx_cv=0;
        kx_dv=0;
        ky_cv=0;
        ky_dv=0;
        k_wz=0.8;
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
        LegCorOri;
        desvxFilt_Old=0;
        desvyFilt_Old=0;
        deswzFilt_Old=0;
        p_ftL_Old=zeros(3,1);
        vNowLF=zeros(3,50);
        pLnorm;
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
            obj.pLnorm=[0;0;-obj.r0(3)]*[1,1,1,1]+reshape([0;1;0;0;-1;0;0;1;0;0;-1;0],3,4)*obj.roll_Off; % norminal foot position in the leg coordinate
            yW=obj.lateral_width;
            xW=obj.sagetial_width;
            %             obj.PendAllnorm=[0.1075,0.1075,-0.1075,-0.1075;
            %                 0.0750,-0.0750,0.0750,-0.0750;
            %                 0,0,0,0];
            PendAlltmp=zeros(12,1);
            PendAlltmp(1:3)=[xW;yW;0]/2;
            PendAlltmp(4:6)=[xW;-yW;0]/2;
            PendAlltmp(7:9)=[-xW;yW;0]/2;
            PendAlltmp(10:12)=[-xW;-yW;0]/2;
            PendAlltmp=PendAlltmp+[0;1;0;0;-1;0;0;1;0;0;-1;0]*obj.roll_Off;
            obj.LegCorOri=reshape(PendAlltmp-[0;1;0;0;-1;0;0;1;0;0;-1;0]*obj.roll_Off,3,4);
        end

        function pL_sw = stepImpl(obj,xFB,xRef,LegState,LegPhase,pL_LF,surP,surVN)
            % surP: origin point on the terrain surface
            % surVN: normal vector of the terrain surface
            pL_LF=reshape(pL_LF,3,4);
            desvX=xRef(7);
            desvY=xRef(8);
            desvZ=xRef(9);
            deswX=xRef(10);
            deswY=xRef(11);
            deswZ=xRef(12);
            desRoll=xRef(4);
            desPit=xRef(5);
            desYaw=xRef(6);
            xFiltFactor=0.9;%0.1
            yFiltFactor=0.9; %0.1
            wzFiltFactor=0.9;
            
            desvxFilt=desvX*xFiltFactor+obj.desvxFilt_Old*(1-xFiltFactor);
            desvyFilt=desvY*yFiltFactor+obj.desvyFilt_Old*(1-yFiltFactor);
            deswzFilt=deswZ*wzFiltFactor+obj.deswzFilt_Old*(1-wzFiltFactor);

            obj.desvxFilt_Old=desvxFilt;
            obj.desvyFilt_Old=desvyFilt;
            obj.deswzFilt_Old=deswzFilt;
            
            vDes=[desvxFilt;desvyFilt;desvZ];
            wDes=[deswX;deswY;deswzFilt];
            RrpyDes=Rz(desYaw)*Ry(desPit)*Rx(desRoll);
            vDesL=RrpyDes'*vDes; % Yet to ADD Rx and Ry !!!!!!!!!!!!!!!!!
            wDesL=RrpyDes'*wDes;
            
            vNow=[xFB(7:8);xFB(9)];
            wNow=[xFB(10);xFB(11);xFB(12);];
            yawNow=xFB(6);
            Rrpy=Rz(yawNow)*Ry(xFB(5))*Rx(xFB(4));
            vNowL=RrpyDes'*vNow; % Yet to ADD Rx and Ry !!!!!!!!!!!!!!!!!
            %wNowL=RrpyDes'*wNow;
            obj.vNowLF(:,1:end-1)=obj.vNowLF(:,2:end);
            obj.vNowLF(:,end)=vNowL;
            vNowL_LF=sum(obj.vNowLF,2)/length(obj.vNowLF(1,:));

            %%% next step foot-placement in the leg coordinate
            
%             px=obj.kx_cv*vNowL_LF(1)*obj.tSW/2+obj.kx_dv*(vNowL_LF(1)-vDesL(1))*obj.tSW/2;
%             py=obj.ky_cv*vNowL_LF(2)*obj.tSW/2+obj.ky_dv*(vNowL_LF(2)-vDesL(2))*obj.tSW/2;

            px=obj.kx_cv*vNowL_LF(1)*obj.tSW/2+obj.kx_dv*vDesL(1)*obj.tSW/2;
            py=obj.ky_cv*vNowL_LF(2)*obj.tSW/2+obj.ky_dv*vDesL(2)*obj.tSW/2;

            p_ftL=[px;py;0];
            obj.p_ftL_Old=p_ftL;

            % rotation compensation
            %sitaZ=0.1*wNowL(3)*obj.T/4+obj.k_wz*(wNowL(3)-wDesL(3))*obj.T/4;
            sitaZ=obj.k_wz*wDesL(3)*obj.tSW/4;
            p_wz1=Rz(sitaZ)*obj.pLnorm-obj.pLnorm;
            p_wz2=0.5*sqrt(0.19/9.8)*cross(vNowL,[0;0;wDesL(3)]);

            % final foot placement
            desAllL=p_ftL*[1,1,1,1]+p_wz1+p_wz2+obj.pLnorm;
            
            % terrain height compensation
            pCoM=[xFB(1);xFB(2);xFB(3)];
            surP_L=Rrpy'*(surP-pCoM);
            surVN_L=Rrpy'*surVN;
            desAllL_z=zeros(4,1);
            for i=1:1:4
                desAllL_z(i)=(surP_L-obj.LegCorOri(:,i))'*surVN_L;
                delta=-obj.r0(3)-desAllL_z(i);
                desAllL_z(i)=desAllL_z(i)+delta*0.8;
            end
            
            %%% next step foot-end position planning in the leg coordinate
            pL_sw=zeros(3,4);
            for i=1:1:4
                if LegState(i)<0.5
                    sLeg=LegPhase(i);
                    sLeg=sLeg*1.1; % to accelerate the swing trajectory tracing
                    if sLeg>=1
                        sLeg=1;
                    end
                    sta=pL_LF(:,i);
                    des=desAllL(:,i);
                    %des(3)=sta(3);
                    %des(3)=-0.19;
                    des(3)=desAllL_z(i);
                    %kv=obj.T/2;
%                     ax=[sta(1),sta(1)+(-vNowL(1))/3*kv,des(1)-(-vNowL(1))/3*kv,des(1)];
%                     ay=[sta(2),sta(2)+(-vNowL(2))/3*kv,des(2)-(-vNowL(2))/3*kv,des(2)];
                    ax=[sta(1),sta(1),des(1),des(1)];
                    ay=[sta(2),sta(2),des(2),des(2)];
                    az1=[sta(3),sta(3),(sta(3)+des(3))/2+obj.StepH,(sta(3)+des(3))/2+obj.StepH];
                    az2=[(sta(3)+des(3))/2+obj.StepH,(sta(3)+des(3))/2+obj.StepH,des(3),des(3)];
                    px=BezierCurve(ax,sLeg);
                    py=BezierCurve(ay,sLeg);
                    pz=0;
                    if sLeg<=0.5
                        pz=BezierCurve(az1,2*sLeg);
                    else
                        pz=BezierCurve(az2,2*(sLeg-0.5));
                    end
                    pL_sw(:,i)=[px;py;pz];
                end
            end

            obj.LegStateOld=LegState;

            pL_sw=reshape(pL_sw,12,1);
        end
        
        function d1 = getOutputDataTypeImpl(~)
            d1 = 'double';
        end

        function s1 = getOutputSizeImpl(~)
            s1 = [12,1];
        end

        function f1 = isOutputFixedSizeImpl(~)
            f1 = true;
        end

        function cpl1 = isOutputComplexImpl(~)
            cpl1 = false;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

    end
end

function M=Rx(sita)
% 3D rotation matrix, vb=M*v:
% rotate a vector in one frame,
% or change the vector 'v' in rotated frame to 'vb' in world frame
M=[1,0,0;
    0,cos(sita),-sin(sita);
    0,sin(sita),cos(sita)];
end

function M=Ry(sita)
% 3D rotation matrix, vb=M*v:
% rotate a vector in one frame,
% or change the vector 'v' in rotated frame to 'vb' in world frame
M=[cos(sita),0,sin(sita);
    0,1,0;
    -sin(sita),0,cos(sita)];
end

function M=Rz(sita)
% 3D rotation matrix, vb=M*v:
% rotate a vector in one frame,
% or change the vector 'v' in rotated frame to 'vb' in world frame
M=[cos(sita),-sin(sita),0;
    sin(sita),cos(sita),0;
    0,0,1];
end

function B=BezierCurve(a,s)
% a is the control point vector, s is the time vector
B=0;
M=length(a);
for ii=1:1:M
    B=B+factorial(M-1)/factorial(ii-1)/factorial(M-ii)*s^(ii-1)*(1-s)^(M-ii)*a(ii);
end
end



