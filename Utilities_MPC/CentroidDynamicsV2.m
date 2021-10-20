classdef CentroidDynamicsV2< matlab.System
    % Centroid Dynamics model
    % X=[r; theta; dr; omega], theta is the ZYX euler angles
    % U=[f1; f2; f3; f4];
    
    properties(Nontunable)
        SampleTime = 0.005; % Sample Time
        OffsetTime = 0; % Offset Time
        TickTime = 0.1;
        startPhase=0;
        SampleTimeTypeProp (1, 1) {mustBeMember(SampleTimeTypeProp, ...
            ["Discrete","FixedInMinorStep","Controllable",...
            "Inherited","InheritedNotControllable",...
            "InheritedErrorConstant"])} = "Discrete"
    end
    
    properties
        m=3.5;
        I=diag([1 1 1]);
        r0=[0;0;0];
        theta0=[0;0;0];
        dr0=[0;0;0];
        omega0=[0;0;0];
    end
    
    properties(Access = private)
        XOld=single(zeros(12,1));
        ROld=single(eye(3));
        yawOff=single(0);
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
            obj.XOld=[obj.r0;obj.theta0;obj.dr0;obj.omega0];
            obj.ROld=Rz(obj.theta0(3))*Ry(obj.theta0(2))*Rx(obj.theta0(1));
            obj.yawOff=0;
        end
        
        function [XNow,ddr,Uin] = stepImpl(obj,U,Pend,LegState,ExtF,ExtTau)
            U=double(U(1:12));
            Uin=reshape(U,3,4);
            Pend=reshape(Pend,3,4);
            
            r=obj.XOld(1:3);
            dr=obj.XOld(7:9);
            omega=obj.XOld(10:12);
            Inow=obj.ROld*obj.I*obj.ROld';
            
            ddr=Uin(:,1)*LegState(1)+Uin(:,2)*LegState(2)+...
                Uin(:,3)*LegState(3)+Uin(:,4)*LegState(4)+[0;0;-obj.m*9.8];
            ddr=(ddr+ExtF)/obj.m;
            temp=-cross(omega,Inow*omega)+cross(Pend(:,1)-r,Uin(:,1))*LegState(1)+...
                cross(Pend(:,2)-r,Uin(:,2))*LegState(2)+cross(Pend(:,3)-r,Uin(:,3))*LegState(3)+...
                cross(Pend(:,4)-r,Uin(:,4))*LegState(4)+ExtTau;
            domega=Inow\temp;
            Rnow=RodFormula(omega*obj.SampleTime)*obj.ROld;
            
            rNew=r+dr*obj.SampleTime;
            thetaNew=Rot2Eul(Rnow);
            if abs(thetaNew(3)-obj.XOld(6)+obj.yawOff)>1.5*pi
                obj.yawOff=obj.yawOff+2*pi*sign(obj.XOld(6)-thetaNew(3)-obj.yawOff);
            end
            thetaNew(3)=thetaNew(3)+obj.yawOff;
            
            drNew=dr+ddr*obj.SampleTime;
            omegaNew=omega+domega*obj.SampleTime;
            XNow=[rNew;thetaNew;drNew;omegaNew];
            
            obj.XOld=XNow;
            obj.ROld=Rnow;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function flag = isInactivePropertyImpl(obj,prop)
            flag = false;
            switch char(obj.SampleTimeTypeProp)
                case {'Inherited', ...
                        'InheritedNotControllable', ...
                        'FixedInMinorStep'}
                    if any(strcmp(prop,{'SampleTime','OffsetTime','TickTime'}))
                        flag = true;
                    end
                case 'Discrete'
                    if any(strcmp(prop,{'TickTime'}))
                        flag = true;
                    end
                case 'Controllable'
                    if any(strcmp(prop,{'SampleTime','OffsetTime'}))
                        flag = true;
                    end
            end
        end
    
    function [d1,d2,d3] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'double';
            d3 = 'double';
    end
    
    function [s1,s2,s3] = getOutputSizeImpl(~)
            s1 = [12,1];
            s2=[3,1];
            s3=[3,4];
        end
        
        function [f1,f2,f3] = isOutputFixedSizeImpl(~)
            f1 = true;
            f2 = true;
            f3=true;
        end
        
        function [cpl1,cpl2,cpl3] = isOutputComplexImpl(~)
            cpl1 = false;
            cpl2=false;
            cpl3=false;
        end
    end
end
%% subfunctions
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

function theta=Rot2Eul(R)
% Rotation matrix to euler angles, ZYX intrinsic order
sy=sqrt(R(1,1)^2+R(2,1)^2);
theta=zeros(3,1);
if sy<1e-6
    theta(1)=atan2(-R(2,3),R(2,2));
    theta(2)=atan2(-R(3,1),sy);
else
    theta(1)=atan2(R(3,2),R(3,3));
    theta(2)=atan2(-R(3,1),sy);
    theta(3)=atan2(R(2,1),R(1,1));
end
end

function R=RodFormula(omega)
% Rodrigues' formula
theta=norm(omega);
if theta>1e-10
    a=omega/theta;
else
    a=omega;
end
aCap=[0,-a(3),a(2);
    a(3),0,-a(1);
    -a(2),a(1),0];
R=eye(3)+aCap*sin(theta)+aCap*aCap*(1-cos(theta));
end







