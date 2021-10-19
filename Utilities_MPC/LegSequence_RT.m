classdef LegSequence_RT< matlab.System
    % adjust supporting legs, foot-hold points, and CoM state for MPC
    properties
        r0=[0;0;0.19];
        theta0=[0;0;0];
        dr0=[0;0;0];
        omega0=[0;0;0];
        lateral_width=0.097;
        sagetial_width=0.2108;
        roll_Off=0.037;
        kx=0;
        ky=0;
        kRz=0;
        SampleTime = 0.005; % Sample Time
        OffsetTime = 0; % Offset Time
        TickTime = 0;
        startPhase=0;
        SampleTimeTypeProp (1, 1) {mustBeMember(SampleTimeTypeProp, ...
            ["Discrete","FixedInMinorStep","Controllable",...
            "Inherited","InheritedNotControllable",...
            "InheritedErrorConstant"])} = "Discrete"
    end
    
    properties(Access=private)
        phiOld;
        PendAllnorm;
        PendAllLS; % PendAll of last step, updates when a leg touches the ground
        PendAllLS_b; % PendAll of last step, updates when a leg leaves the ground
        rLS_b; % CoM position of last step, updates when a leg leaves the ground
        PendAllOld;
        PendAllLocalOld;
        LegStateOld;
        X_mpc_Old;
        X_interPol;
        RateCount;
    end
    
    methods(Access = protected)
        function sts = getSampleTimeImpl(obj)
            switch char(obj.SampleTimeTypeProp)
                case 'Inherited'
                    sts = createSampleTime(obj,'Type','Inherited');
                case 'InheritedNotControllable'
                    sts = createSampleTime(obj,'Type','Inherited',...
                        'AlternatePropagation','Controllable');
                case 'InheritedErrorConstant'
                    sts = createSampleTime(obj,'Type','Inherited',...
                        'ErrorOnPropagation','Constant');
                case 'FixedInMinorStep'
                    sts = createSampleTime(obj,'Type','Fixed In Minor Step');
                case 'Discrete'
                    sts = createSampleTime(obj,'Type','Discrete',...
                        'SampleTime',obj.SampleTime, ...
                        'OffsetTime',obj.OffsetTime);
                case 'Controllable'
                    sts = createSampleTime(obj,'Type','Controllable',...
                        'TickTime',obj.TickTime);
            end
        end
        
        function setupImpl(obj)
            obj.phiOld=0;
            xW=obj.lateral_width;
            yW=obj.sagetial_width;
            %             obj.PendAllnorm=[0.1075,0.1075,-0.1075,-0.1075;
            %                 0.0750,-0.0750,0.0750,-0.0750;
            %                 0,0,0,0];
            PendAlltmp=zeros(12,1);
            PendAlltmp(1:3)=[xW;yW;0]/2;
            PendAlltmp(4:6)=[xW;-yW;0]/2;
            PendAlltmp(7:9)=[-xW;yW;0]/2;
            PendAlltmp(10:12)=[-xW;-yW;0]/2;
            PendAlltmp=PendAlltmp+[0;1;0;0;-1;0;0;1;0;0;-1;0]*obj.roll_Off;
            obj.PendAllnorm=reshape(PendAlltmp,3,4);
            obj.PendAllLS=obj.PendAllnorm;
            obj.PendAllLS_b=obj.PendAllnorm;
            obj.rLS_b=obj.r0*[1,1,1,1];
            obj.PendAllOld=obj.PendAllnorm;
            obj.PendAllLocalOld=zeros(3,4);
            obj.LegStateOld=[1;0;0;1];
            obj.X_mpc_Old=zeros(12,1);
            obj.RateCount=0;
            obj.X_interPol=[obj.r0;zeros(9,1)];
        end
        
        function [LegState,PendAll,PendAllLocal,PendAllLs,desAllV1,desAllV2] = stepImpl(obj,phi,X_FB,X_mpc,T,StepH,touchInd,ref,surP,zSur,AdX,BdX,U)
            % T is the moving period
            % PendAllLs: last support point, update when a leg touches the ground
            
            LegState=[1;0;0;1];
            s=phi/pi;
            if phi>pi
                LegState=[0;1;1;0];
                s=s-1;
            end
            desvX=ref(7);
            desvY=ref(8);
            desvZ=ref(9);
            desRoll=ref(4);
            desPit=ref(5);
            desYaw=ref(6);
            vDes=[desvX;desvY;desvZ];
            vDesL=Rz(desYaw)'*vDes;
            
            vNow=X_FB(7:9);
            yawNow=X_FB(6);
            vNowL=Rz(yawNow)'*vNow;
            v=vNowL+[obj.kx*(vDesL(1)-vNowL(1)); ...
                obj.ky*(vDesL(2)-vNowL(2));0];
            
            desAllL=v*T/4*[1,1,1,1]+obj.PendAllnorm;
            desAll=Rz(desYaw)*Ry(desPit)*Rx(desRoll)*desAllL+[X_FB(1);X_FB(2);0]*[1,1,1,1];
            
            desAllV2=vDes*T/2*[1,1,1,1]+obj.PendAllLS_b+[X_FB(1);X_FB(2);0]*[1,1,1,1]-obj.rLS_b;
            desAllV1=desAll;
            
            desAll=desAllV1;
            for i=1:1:4
                desAll(3,i)=[1,desAll(1,i),desAll(2,i)]*[surP(1);surP(2);surP(3)];
            end
            
            %desAll=obj.PendAllnorm;
            
            PendAll=zeros(3,4);
            for i=1:1:4
                if LegState(i)<0.5
                    sta=obj.PendAllLS(:,i);
                    des=desAll(:,i);
                    ax=[sta(1),sta(1),des(1),des(1)];
                    ay=[sta(2),sta(2),des(2),des(2)];
                    az1=[sta(3),sta(3),(sta(3)+des(3))/2+StepH,(sta(3)+des(3))/2+StepH];
                    az2=[(sta(3)+des(3))/2+StepH,(sta(3)+des(3))/2+StepH,des(3),des(3)];
                    px=BezierCurve(ax,s);
                    py=BezierCurve(ay,s);
                    pz=0;
                    if s<=0.5
                        pz=BezierCurve(az1,2*s);
                    else
                        pz=BezierCurve(az2,2*(s-0.5));
                    end
                    PendAll(:,i)=[px;py;pz];
                    if touchInd(i)>0.5
                        PendAll(:,i)=obj.PendAllOld(:,i);
                        PendAll(3,i)=zSur(i);
                    end
                elseif LegState(i)>0.5 && obj.LegStateOld(i)<0.5
                    PendAll(:,i)=obj.PendAllOld(:,i);
                    PendAll(3,i)=zSur(i);
                    obj.PendAllLS(:,i)=PendAll(:,i);
                else
                    PendAll(:,i)=obj.PendAllLS(:,i);
                end
                if LegState(i)<0.5 && obj.LegStateOld(i)>0.5
                    obj.PendAllLS_b(:,i)=PendAll(:,i);
                    obj.rLS_b(:,i)=reshape(X_FB(1:3),3,1);
                    obj.rLS_b(3,i)=0;
                end
            end
            X_mpc_est=X_mpc(1,:)';
            if sum(abs(obj.X_mpc_Old-X_mpc_est(1:12)))<10^-5
                %theta=obj.X_mpc_Old(4:6);%+obj.RateCount*obj.SampleTime*obj.X_mpc_Old(10:12)+0.5*(obj.RateCount*obj.SampleTime)^2*dX(10:12);
                %rNow=obj.X_mpc_Old(1:3)+obj.RateCount*obj.SampleTime*obj.X_mpc_Old(7:9)+0.5*(obj.RateCount*obj.SampleTime)^2*dX(7:9);
                
                Xnow=AdX*[obj.X_interPol;9.8]+BdX*U;
                theta=Xnow(4:6);
                rNow=Xnow(1:3);
                R=Rz(theta(3))*Ry(theta(2))*Rx(theta(1));
                PendAllLocal=R'*(PendAll-rNow*[1,1,1,1])-obj.PendAllnorm;
                obj.RateCount=obj.RateCount+1;
            else
                Xnow=X_mpc_est;
                theta=Xnow(4:6);
                rNow=Xnow(1:3);
                R=Rz(theta(3))*Ry(theta(2))*Rx(theta(1));
                PendAllLocal=R'*(PendAll-rNow*[1,1,1,1])-obj.PendAllnorm;
                obj.RateCount=0;
            end
            PendAllLs=obj.PendAllLS;
            
            obj.X_interPol=Xnow(1:12);
            obj.X_mpc_Old=X_mpc_est(1:12);
            obj.phiOld=phi;
            obj.LegStateOld=LegState;
            obj.PendAllOld=PendAll;
            obj.PendAllLocalOld=PendAllLocal;
        end
        
        function [d1,d2,d3,d4,d5,d6] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'double';
            d3 = 'double';
            d4 = 'double';
            d5='double';
            d6='double';
        end
        
        function [s1,s2,s3,s4,s5,s6] = getOutputSizeImpl(~)
            s1 = [4,1];
            s2=[3,4];
            s3=[3,4];
            s4=[3,4];
            s5=[3,4];
            s6=[3,4];
        end
        
        function [f1,f2,f3,f4,f5,f6] = isOutputFixedSizeImpl(~)
            f1 = true;
            f2 = true;
            f3=true;
            f4=true;
            f5=true;
            f6=true;
        end
        
        function [cpl1,cpl2,cpl3,cpl4,cpl5,cpl6] = isOutputComplexImpl(~)
            cpl1 = false;
            cpl2=false;
            cpl3=false;
            cpl4=false;
            cpl5=false;
            cpl6=false;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end

function R=RodFormula(omega)
% Rodrigues' formula
theta=norm(omega);
if theta>1e-11
    a=omega/theta;
else
    a=omega;
end
aCap=[0,-a(3),a(2);
    a(3),0,-a(1);
    -a(2),a(1),0];
R=eye(3)+aCap*sin(theta)+aCap*aCap*(1-cos(theta));
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








