classdef KalmanFilter_DIY_compound_eqProj < matlab.System
    % Coventional kalman filter for compound state estimation, offset variable are included. Equality constraint for leg state is included.
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
        
        function [xwave,P] = stepImpl(obj,u,y,x0,p0,Q,R,Reset,updateEN,SPLeg)
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
            countD=0;
            for i=1:1:4
                if SPLeg(i)>0.5 && Xpre(3*i+6)<0
                    countD=countD+1;
                end
            end
            D=zeros(countD,21);
            d=zeros(countD,1);
            rawCount=1;
            if countD>0.5
                for i=1:1:4
                    if SPLeg(i)>0.5 && Xpre(3*i+6)<0
                        D(rawCount,:)=[zeros(1,3*i+3),[0,0,1],zeros(1,15-3*i)];
                        %d(rawCount)=obj.XOld(3*i+6);
                        d(rawCount)=0;
                        rawCount=rawCount+1;
                    end
                end
            end

            Xpre=obj.A*obj.XOld+obj.B*u;
            Ppre=obj.A*obj.POld*obj.A'+Qnow;
            K=Ppre*obj.C'*pinv(obj.C*Ppre*obj.C'+Rnow);

%             rk=y-obj.C*Xpre;
%             Sk=obj.C*Ppre*obj.C'+Rnow;
%             xk=Xpre+K*rk;
%             Kwave=K-D'*pinv(D*D')*(D*xk-d)*pinv(rk'*pinv(Sk)*rk)*rk'*pinv(Sk);
%             K=Kwave;

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

            xwave=xhat-D'*pinv(D*D')*(D*xhat-d);
            obj.XOld=xhat;
            obj.POld=P;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end