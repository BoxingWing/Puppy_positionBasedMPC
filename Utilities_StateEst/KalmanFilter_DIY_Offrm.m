classdef KalmanFilter_DIY_Offrm < matlab.System
    % Coventional kalman filter for single-rigid-body state estimation, offset variable are included.
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
            obj.A=[eye(3),eye(3)*obj.Ts,0.5*obj.Ts^2*eye(3);
                zeros(3,3),eye(3),obj.Ts*eye(3);
                zeros(3,3),zeros(3,3),eye(3,3)
                ];
            obj.B=[0.5*obj.Ts^2*eye(3);obj.Ts*eye(3);zeros(3,3)];
            obj.C=[eye(3),zeros(3,3),zeros(3,3);
                        zeros(3,3),eye(3),zeros(3,3);
            ];
            obj.count=0;
            [nX,~]=size(obj.A);
            obj.XOld=zeros(nX,1);
            obj.POld=eye(nX);
            obj.G=eye(nX);
        end
        
        function [xhat,P] = stepImpl(obj,u,y,x0,p0,Q,R,Reset,updateEN)
            % note Q, R must be matrixes
            % Q for process noise
            % R for measurement noise
            if obj.count<0.5
                obj.XOld=x0;
                obj.POld=p0;
                obj.count=obj.count+1;
            end
            Xpre=obj.A*obj.XOld+obj.B*u;
            Ppre=obj.A*obj.POld*obj.A'+obj.G*Q*obj.G';
            K=Ppre*obj.C'/(obj.C*Ppre*obj.C'+R);
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