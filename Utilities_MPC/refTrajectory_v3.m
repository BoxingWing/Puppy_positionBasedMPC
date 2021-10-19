classdef refTrajectory_v3< matlab.System
    % generate desired state values for MPC based on feedback states
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
        sitaZOld;
    end
    properties(Nontunable)
        numP=20; % prediction horizon length
    end
    methods(Access = protected)
        
        function setupImpl(obj)
            obj.refOld=ones(obj.numP,1)*[obj.r0;obj.theta0;obj.dr0;obj.omega0;9.8]';
            obj.sitaErrOld=[0;0;0];
            obj.sitaZOld=0;
        end
        
        function [refSeqOut,refP,refSeq,sitaErr,sitaNow] = stepImpl(obj,vxL,vyL,omegaZ,surVN,sura,X_FB,disable)
            % refSeqOut: ref sequence including sitaErr compensation
            % refSeq: ref sequence excluding sitaErr compensation
            % refP: last element of the refSeq
            vxL=vxL*1;
            vyL=vyL*1;
            
            ref=reshape(X_FB,1,13); ref(13)=9.8;
            refSeq=zeros(obj.numP,13);
            refSeq(1,:)=ref;
            desH=obj.desHeight;
            sitaErr=diag(obj.sitaErr_K)*(obj.refOld(2,4:6)'-reshape(X_FB(4:6),3,1));
            if disable<0.5
                sitaNow=obj.sitaErrOld+sitaErr;
            else
                sitaNow=obj.sitaErrOld;
            end
            
            %             surVN=[0;0;1];
            %             sura=[0;0;0];
            obj.sitaZOld=obj.sitaZOld+omegaZ*obj.dt;
            %Rbody=Rz(X_FB(6))*Ry(X_FB(5))*Rx(X_FB(4));
            Rbody=Ry(X_FB(5))*Rx(X_FB(4));
            headX=Rz(obj.sitaZOld)*Rbody*[1;0;0];
            
            surVN=surVN/norm(surVN);
            surVX=headX-surVN'*headX*surVN;
            surVX=surVX/norm(surVX);
            surVY=cross(surVN,surVX);
            Rsur=[surVX,surVY,surVN];
            
            for i=2:1:obj.numP
                desTheta=Rot2Eul(Rsur);
                if i<1.5
                    desr=Rsur*[vxL;vyL;0]*obj.dt+reshape(X_FB(1:3),3,1);
                else
                    desr=Rsur*[vxL;vyL;0]*obj.dt+refSeq(i-1,1:3)';
                end
                desdr=Rsur*[vxL;vyL;0];
                desdtheta=Rsur*[0;0;omegaZ];
                pSur=[0;0;sura(1)]; % get the point on the surface whose x and y coordinates are zero
                h=(desr-pSur)'*surVN;
                desr=desr+(desH-h)*surVN; % guarantee that the distance between the height and the robot are the desired height
                
                refSeq(i,1:3)=desr;
                refSeq(i,4:6)=desTheta;
                refSeq(i,7:9)=desdr;
                refSeq(i,10:12)=desdtheta;
                refSeq(i,13)=9.8;
            end
            
            if disable>0.5
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

