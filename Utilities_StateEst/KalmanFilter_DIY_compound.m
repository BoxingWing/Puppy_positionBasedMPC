classdef KalmanFilter_DIY_compound < matlab.System
    % Coventional kalman filter for compound state estimation, offset variable are included.
    properties
        Ts=0.005;
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
            % x=[r,v,p1,p2,p3,p4,aBar];
            % y=[prel1,prel2,prel3,prel4,prel1z,prel2z,prel3z,prel4z,v]

            obj.A=[eye(3),eye(3)*obj.Ts,zeros(3,12),0.5*obj.Ts^2*eye(3);
                zeros(3,3),eye(3),zeros(3,12),eye(3)*obj.Ts;
                zeros(3,6),eye(3),zeros(3,12);
                zeros(3,9),eye(3),zeros(3,9);
                zeros(3,12),eye(3),zeros(3,6);
                zeros(3,15),eye(3),zeros(3,3);
                zeros(3,18),eye(3);
                ];

            obj.B=[0.5*obj.Ts^2*eye(3);obj.Ts*eye(3);zeros(15,3)];

            M=[0,0,1];
            obj.C=[-eye(3),zeros(3,3),eye(3),zeros(3,12);
                -eye(3),zeros(3,6),eye(3),zeros(3,9);
                -eye(3),zeros(3,9),eye(3),zeros(3,6);
                -eye(3),zeros(3,12),eye(3),zeros(3,3);
                zeros(1,6),M,zeros(1,12);
                zeros(1,9),M,zeros(1,9);
                zeros(1,12),M,zeros(1,6);
                zeros(1,15),M,zeros(1,3);
                zeros(3,3),eye(3),zeros(3,15);
            ];

            obj.count=0;
            [nX,~]=size(obj.A);
            obj.XOld=zeros(nX,1);
            obj.POld=eye(nX);
        end
        
        function [xhat,P] = stepImpl(obj,u,y,x0,p0,Q,R,Reset,updateEN)
            % note Q, R must be matrixes
            % Q for process noise
            % R for measurement noise
            Qnow=diag(Q);
            Rnow=diag(R);
            if obj.count<0.5
                obj.XOld=x0;
                obj.POld=p0;
                obj.count=obj.count+1;
            end
            Xpre=obj.A*obj.XOld+obj.B*u;
            Ppre=obj.A*obj.POld*obj.A'+Qnow;
            K=Ppre*obj.C'/(obj.C*Ppre*obj.C'+Rnow);
            P=Ppre;
            if updateEN<0.5
                K=K*0;
            else
                P=(eye(length(x0))-K*obj.C)*Ppre;
            end
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