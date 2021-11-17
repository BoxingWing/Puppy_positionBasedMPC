classdef refTrajectory_v4< matlab.System
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
        refSeqOld;
        sitaErrOld;
        sitaZOld;
        pxOld;
        pyOld;
    end
    properties(Nontunable)
        numP=20; % prediction horizon length
    end
    methods(Access = protected)
        
        function setupImpl(obj)
            obj.refSeqOld=ones(obj.numP,1)*[obj.r0;obj.theta0;obj.dr0;obj.omega0;9.8]';
            obj.sitaErrOld=[0;0;0];
            obj.sitaZOld=0;
            obj.pxOld=0;
            obj.pyOld=0;
        end
        
        function [refSeqOut,refP,refSeq,sitaErr,sitaNow] = stepImpl(obj,vxL,vyL,omegaZ,surVN,sura,X_FB,disable)
            % refSeqOut: ref sequence including sitaErr compensation
            % refSeq: ref sequence excluding sitaErr compensation
            % refP: ref of the first prediciton horizon excluding sitaErr
            % disable: sitaErr will not be accumulated, ref will not be updated
            vxL=vxL*1;
            vyL=vyL*1;
            
            curState=X_FB(1:13); curState(13)=9.8;
            refSeq=zeros(obj.numP,13);
            refSeq(1,:)=curState;
            desH=obj.desHeight;
            sitaErr=obj.refSeqOld(2,4:6)'-reshape(X_FB(4:6),3,1);
            rolDZ=0.5/180*pi; % deadzone
            pitDZ=0.5/180*pi;
            yawDZ=1/180*pi;
            if sitaErr(1)>abs(rolDZ) || sitaErr(1)<-abs(rolDZ)
                sitaErr(1)=sitaErr(1)-sign(sitaErr(1))*abs(rolDZ);
            else
                sitaErr(1)=0;
            end
            if sitaErr(2)>abs(pitDZ) || sitaErr(2)<-abs(pitDZ)
                sitaErr(2)=sitaErr(2)-sign(sitaErr(2))*abs(pitDZ);
            else
                sitaErr(2)=0;
            end
            if sitaErr(3)>abs(yawDZ) || sitaErr(3)<-abs(yawDZ)
                sitaErr(3)=sitaErr(3)-sign(sitaErr(3))*abs(yawDZ);
            else
                sitaErr(3)=0;
            end
            if disable<0.5
                sitaNow=obj.sitaErrOld+diag(obj.sitaErr_K)*sitaErr;
            else
                sitaNow=obj.sitaErrOld;
            end
            if disable>0.5
                obj.pxOld=0;
                obj.pyOld=0;
                obj.sitaZOld=0;
                obj.sitaErrOld=[0;0;0];
            end
            
            %             surVN=[0;0;1];
            %             sura=[0;0;0];
            obj.sitaZOld=obj.sitaZOld+omegaZ*obj.dt;
            obj.pxOld=obj.pxOld+vxL*obj.dt;
            obj.pyOld=obj.pyOld+vyL*obj.dt;
            %Rbody=Rz(X_FB(6))*Ry(X_FB(5))*Rx(X_FB(4));
            Rbody=Ry(X_FB(5))*Rx(X_FB(4));
            headX=Rz(obj.sitaZOld)*Rbody*[1;0;0];
            
            surVN=surVN/norm(surVN);
            surVX=headX-surVN'*headX*surVN;
            surVX=surVX/norm(surVX);
            surVY=cross(surVN,surVX);
            RwOri=[surVX,surVY,surVN];
            
            %%% leave the first step ref to the current state
%             for i=2:1:obj.numP
%                 desTheta=Rot2Eul(Rsur);
%                 if i<2.5
%                     desr=Rsur*[vxL;vyL;0]*obj.dt+obj.refSeqOld(3,1:3)';
%                 else
%                     desr=Rsur*[vxL;vyL;0]*obj.dt+refSeq(i-1,1:3)';
%                 end
%                 desdr=Rsur*[vxL;vyL;0];
%                 desdtheta=Rsur*[0;0;omegaZ];
%                 pSur=[0;0;sura(1)]; % get the point on the surface whose x and y coordinates are zero
%                 h=(desr-pSur)'*surVN;
%                 desr=desr+(desH-h)*surVN; % guarantee that the distance between the height and the robot are the desired height
%                 
%                 refSeq(i,1:3)=desr;
%                 refSeq(i,4:6)=desTheta;
%                 refSeq(i,7:9)=desdr;
%                 refSeq(i,10:12)=desdtheta;
%                 refSeq(i,13)=9.8;
%             end
            
            %%% override the first step ref to the desired ref
            for i=1:1:obj.numP
                Rbody=Rodrigues(surVN*omegaZ*obj.dt*(i-1))*RwOri;
                desTheta=Rot2Eul(Rbody);
                desTheta(3)=obj.sitaZOld+omegaZ*obj.dt*(i-1);
                if i<1.5
                    desr=[obj.pxOld;obj.pyOld;obj.r0(3)];
                else
                    desr=Rbody*[vxL;vyL;0]*obj.dt+refSeq(i-1,1:3)';
                end
                desdr=Rbody*[vxL;vyL;0];
                desdtheta=Rbody*[0;0;omegaZ];
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
                refSeq=obj.refSeqOld;
            end
            refP=refSeq(1,:);
            
            refSeqOut=refSeq;
            for i=1:1:obj.numP
                refSeqOut(i,:)=refSeqOut(i,:)+[zeros(3,1);sitaNow;zeros(7,1)]';
            end
            
            if disable>0.5
                refSeqOut=refSeq;
            end
            
            obj.refSeqOld=refSeq;
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

function R=Rodrigues(w)
% Rodrigues' formula
if abs(norm(w))<10^-5
    a=[1;0;0];
else
    a=w/norm(w);
end
sita=norm(w);
R=eye(3)+cap(a)*sin(sita)+cap(a)*cap(a)*(1-cos(sita));
end

%%%%% subfunction
function capM=cap(w)
capM=[0,-w(3),w(2);
    w(3),0,-w(1);
    -w(2),w(1),0];
end