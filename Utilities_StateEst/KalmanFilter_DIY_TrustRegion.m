classdef KalmanFilter_DIY_TrustRegion < matlab.System
    % Coventional kalman filter for single-rigid-body state estimation, offset variable are included.
    properties
        dt=0.006;
        Q_wPCoM=[1e-7;5e-8;1e-8];
        Q_wVCoM=[1e-7,1e-9,1e-5];
        Q_wfeW=ones(12,1)*(1e-6);
        R_wfeL=ones(12,1)*(1e-6);
        R_wdfeL=ones(12,1)*(1e-3);
    end
    
    properties (Access=private)
        A;
        B;
        C;
        G;
        XOld=zeros(18,1);
        POld=zeros(18,18);
        count;
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            blkA=[eye(3),obj.dt*eye(3);zeros(3,3),eye(3)];
            obj.A=[blkA,zeros(6,12);zeros(12,6),eye(12)];
            %obj.B=[0.5*obj.dt^2*eye(3);obj.dt*eye(3);zeros(12,3)];
            obj.B=[0*eye(3);obj.dt*eye(3);zeros(12,3)];
            blkC1=[-eye(3),zeros(3,3);
                -eye(3),zeros(3,3);
                -eye(3),zeros(3,3);
                -eye(3),zeros(3,3);];
            blkC2=[zeros(3,3),-eye(3);
                zeros(3,3),-eye(3);
                zeros(3,3),-eye(3);
                zeros(3,3),-eye(3);];
            obj.C=[blkC1,eye(12);blkC2,zeros(12,12)];
            obj.count=0;
            [nX,~]=size(obj.A);
            obj.XOld=zeros(nX,1);
            obj.POld=eye(nX);
            obj.G=eye(nX);
        end
        
        function [xhat,P] = stepImpl(obj,u,y,x0,p0,Xi,Reset)
            % note Q, R must be matrixes
            % Q for process noise
            % R for measurement noise
            if obj.count<0.5
                obj.XOld=x0;
                obj.POld=p0;
                obj.count=2;
            end
            blkQ1=diag(obj.Q_wPCoM)*eye(3);
            blkQ2=diag(obj.Q_wVCoM)*eye(3);
            blkQ3=diag(obj.Q_wfeW)*Xi*eye(12);
            Q=blkdiag(blkQ1,blkQ2,blkQ3);
            blkR1=diag(obj.R_wfeL)*Xi*eye(12);
            blkR2=diag(obj.R_wdfeL)*Xi*eye(12);
            R=blkdiag(blkR1,blkR2);
            
            Xpre=obj.A*obj.XOld+obj.B*u;
            Ppre=obj.A*obj.POld*obj.A'+obj.G*Q*obj.G';
            Sinv=inv(obj.C*Ppre*obj.C'+R);
            K=Ppre*obj.C'*Sinv;
            
            P=(eye(length(x0))-K*obj.C)*Ppre;

            xhat=Xpre+K*(y-obj.C*Xpre);
            if Reset>0.5
                xhat=x0;
                P=p0;
            end
            obj.XOld=xhat;
            obj.POld=P;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end