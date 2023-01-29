classdef KalmanFilter_DIY_TrustRegion_h < matlab.System
    % Coventional kalman filter for single-rigid-body state estimation, offset variable are included.
    properties
        dt=0.006;
        Q_wPCoM=[1e-7;5e-8;1e-8];
        Q_wVCoM=[1e-7,1e-9,1e-5];
        Q_wfeW=ones(12,1)*(1e-6);
        Q_waL=ones(3,1)*(1e-3);
        R_wfeL=ones(12,1)*(1e-6);
        R_wdfeL=ones(12,1)*(1e-3);
        R_wh=ones(4,1)*(1e-6);
    end
    
    properties (Access=private)
        A;
        B;
        C;
        G;
        XOld;
        POld;
        count;
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            blkA=[eye(3),obj.dt*eye(3);zeros(3,3),eye(3)];
            blkA_a=[0*0.5*eye(3)*(obj.dt)^2;eye(3)*obj.dt];
            obj.A=[blkA,zeros(6,12),blkA_a;
                zeros(12,6),eye(12),zeros(12,3);
                zeros(3,18),eye(3)];
            %obj.B=[0.5*obj.dt^2*eye(3);obj.dt*eye(3);zeros(12,3)];
            obj.B=[0.5*obj.dt^2*eye(3);obj.dt*eye(3);zeros(12,3);zeros(3,3)];
            blkC1=[-eye(3),zeros(3,3);
                -eye(3),zeros(3,3);
                -eye(3),zeros(3,3);
                -eye(3),zeros(3,3);];
            blkC2=[zeros(3,3),-eye(3);
                zeros(3,3),-eye(3);
                zeros(3,3),-eye(3);
                zeros(3,3),-eye(3);];
            blkCa1=[-0.5*eye(3)*(obj.dt)^2*0;
                -0.5*eye(3)*(obj.dt)^2*0;
                -0.5*eye(3)*(obj.dt)^2*0;
                -0.5*eye(3)*(obj.dt)^2*0
                ];
            blkCa2=[-eye(3)*obj.dt;
                -eye(3)*obj.dt;
                -eye(3)*obj.dt;
                -eye(3)*obj.dt
                ];
            e3=zeros(1,12);e3(3)=1;
            e6=zeros(1,12);e6(6)=1;
            e9=zeros(1,12);e9(9)=1;
            e12=zeros(1,12);e12(12)=1;
            obj.C=[blkC1,eye(12),blkCa1;blkC2,zeros(12,12),blkCa2; ...
                zeros(4,3),zeros(4,3),[e3;e6;e9;e12],zeros(4,3)];
            obj.count=0;
            [nX,~]=size(obj.A);
            obj.XOld=zeros(nX,1);
            obj.POld=eye(nX);
            obj.G=eye(nX);
        end
        
        function [xhat,P] = stepImpl(obj,u,y,x0,p0,Xi,Xih,Reset)
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
            blkQ4=diag(obj.Q_waL);
            Q=blkdiag(blkQ1,blkQ2,blkQ3,blkQ4);
            blkR1=diag(obj.R_wfeL)*Xi*eye(12);
            blkR2=diag(obj.R_wdfeL)*Xi*eye(12);
            blkR3=diag(obj.R_wh)*Xih*eye(4);
            R=blkdiag(blkR1,blkR2,blkR3);
            
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