classdef IMU_regulation < matlab.System
    % IMU output regulator, the output acc is the decoupled linear acc. All outputs are in the world coordinate. omega and sita are in radian.
    properties (Access=private)
        Roff=eye(3);
        accOff=zeros(3,1);
        sitaOld=zeros(3,1);
        yawOff=0;
        accStore=zeros(3,400);
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end
        
        function [accOffnow,accL_W,omega_W,RPY] = stepImpl(obj,acc,omega,rpy,OffEN)
            % the output accL_W is the decoupled linear acc
            % acc and omega is in the world coordinate
            % omega is in radian
            RPY=rpy;
            MRx=Rx(RPY(1));
            MRy=Ry(RPY(2));
            MRz=Rz(RPY(3));
            Gnow=(MRz*MRy*MRx)'*[0;0;-9.8];
            acc=acc+Gnow;
            if OffEN>0.5
                obj.Roff=MRz';
                obj.accOff(1)=-sum(obj.accStore(1,:))/length(obj.accStore(1,:));
                obj.accOff(2)=-sum(obj.accStore(2,:))/length(obj.accStore(1,:));
                obj.accOff(3)=-sum(obj.accStore(3,:))/length(obj.accStore(1,:));
            end
            Rnow=obj.Roff*(MRz*MRy*MRx);
            RPY=Rot2Eul(Rnow);
            accL_W=Rnow*(acc+obj.accOff);
            omega_W=Rnow*omega;
            %%% expand the yaw angle into endless radian angle
            if abs(RPY(3)-obj.sitaOld(3)+obj.yawOff)>1.5*pi && OffEN<0.5
                obj.yawOff=obj.yawOff+2*pi*sign(obj.sitaOld(3)-RPY(3)-obj.yawOff);
            elseif OffEN>0.5
                obj.yawOff=0;
            end
            RPY(3)=RPY(3)+obj.yawOff;
            accOffnow=obj.accOff;
            
            obj.sitaOld=RPY;
            obj.accStore(:,2:end)=obj.accStore(:,1:end-1);
            obj.accStore(:,1)=acc;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
%         %% Define output properties
        function num = getNumOutputsImpl(~)
            num = 4;
        end
        
        function flag = isOutputSizeLockedImpl(~,~)
            flag{1} = true;
            flag{2} = true;
            flag{3} = true;
            flag{4} = true;
        end
        
        function varargout = isOutputFixedSizeImpl(~,~)
            varargout{1} = true;
            varargout{2} = true;
            varargout{3} = true;
            varargout{4} = true;
        end
        
        function flag = isOutputComplexityLockedImpl(~,~)
            flag{1} = true;
            flag{2} = true;
            flag{3} = true;
            flag{4} = true;
        end
%         
        function varargout = isOutputComplexImpl(~)
            varargout{1} = false;
            varargout{2} = false;
            varargout{3} = false;
            varargout{4} = false;
        end
%         
        function varargout = getOutputSizeImpl(~)
            varargout{1} = [3,1];
            varargout{2} = [3,1];
            varargout{3} = [3,1];
            varargout{4} = [3,1];
        end
%         
        function varargout = getOutputDataTypeImpl(~)
            varargout{1} = 'double';
            varargout{2} = 'double';
            varargout{3} = 'double';
            varargout{4} = 'double';
        end
    end
end
function M=Rx(sita)
% 3D rotation matrix, from world to body
M=[1,0,0;
    0,cos(sita),-sin(sita);
    0,sin(sita),cos(sita)];
end

function M=Ry(sita)
% 3D rotation matrix, from world to body
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