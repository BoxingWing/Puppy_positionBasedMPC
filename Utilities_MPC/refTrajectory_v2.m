classdef refTrajectory_v2< matlab.System
    % generate desired state values for MPCb based on desired states
    properties
        r0=[0;0;0.19];
        theta0=[0;0;0];
        dr0=[0;0;0];
        omega0=[0;0;0];
        desHeight=0.19; % desired CoM height
        sitaErr_K=[0;0;0];
        dt=0.005;
    end
    
    properties(Access=private)
        refOld;
        sitaErrOld;
    end
    properties(Nontunable)
        numP=20; % prediction horizon length
    end
    methods(Access = protected)
        
        function setupImpl(obj)
            obj.refOld=ones(obj.numP,1)*[obj.r0;obj.theta0;obj.dr0;obj.omega0;9.8]';
            obj.sitaErrOld=[0;0;0];
        end
        
        function [refSeqOut,refP,refSeq,sitaErr,sitaNow] = stepImpl(obj,vxL,vyL,omegaZ,surVN,surP,X_FB,disable)
            ref=obj.refOld(end,:);
            desH=obj.desHeight;
            sitaErr=diag(obj.sitaErr_K)*(obj.refOld(end,4:6)'-reshape(X_FB(4:6),3,1));
            if disable<0.5
                sitaNow=obj.sitaErrOld+sitaErr;
            else
                sitaNow=obj.sitaErrOld;
            end
            
            Rbody=Rz(ref(6))*Ry(ref(5))*Rx(ref(4));
            headX=Rz(omegaZ*obj.dt)*Rbody*[1;0;0];
            
            surVX=headX-surVN'*headX*surVN;
            surVX=surVX/norm(surVX);
            surVY=cross(surVN,surVX);
            Rsur=[surVX,surVY,surVN];
            desTheta=Rot2Eul(Rsur);
            desr=Rsur*[vxL;vyL;0]*obj.dt+ref(1:3)';
            desdr=Rsur*[vxL;vyL;0];
            desdtheta=Rsur*[0;0;omegaZ];
            pSur=[0;0;surP(1)]; % get the point on the surface whose x and y coordinates are zero
%             tmp=[1,desr(1),desr(2)]*reshape(surP,3,1);
%             pSur=[desr(1);desr(2);tmp];
            h=(desr-pSur)'*surVN;
            desr=desr+(desH-h)*surVN; % guarantee that the distance between the height and the robot are the desired height
            
            ref(1:3)=desr;
            ref(4:6)=desTheta;
            ref(7:9)=desdr;
            ref(10:12)=desdtheta;
            ref(13)=9.8;
            
            if disable<0.5
                refSeq=[obj.refOld(2:end,:);ref];
            else
                refSeq=obj.refOld;
            end
            refP=refSeq(end,:);
            
            refSeqOut=refSeq;
            for i=1:1:obj.numP
                refSeqOut(i,:)=refSeqOut(i,:)+[zeros(3,1);sitaNow;zeros(7,1)]';
            end
            
            if disable>0.5
                refSeqOut=refSeq;
            end
            
            obj.refOld=refSeq;
            obj.sitaErrOld=sitaNow;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end
%% subfunction
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

