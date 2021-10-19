classdef KalmanFilter_DIY_yaw< matlab.System
    % Coventional kalman filter
    properties
        A=1;
        B=1;
        C=1;
        G=1;
        x0=0;
        p0=1;
        Q=0.1;
        R=0.5;
    end
    
    properties (Access=private)
        XOld;
        POld;
        count;
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.count=0;
            [nX,~]=size(obj.A);
            obj.XOld=zeros(nX,1);
            obj.POld=eye(nX);
        end
        
        function xhat = stepImpl(obj,u,y,Reset,updateEN)
            % note Q, R must be matrixes
            % Q for process noise
            % R for measurement noise
            if obj.count<0.5
                obj.XOld=obj.x0;
                obj.POld=obj.p0;
                obj.count=obj.count+1;
            end
            Xpre=obj.A*obj.XOld+obj.B*u;
            Ppre=obj.A*obj.POld*obj.A'+obj.G*obj.Q*obj.G';
            K=Ppre*obj.C'/(obj.C*Ppre*obj.C'+obj.R);
            P=Ppre;
            if updateEN<0.5
                K=K*0;
            else
                P=(eye(length(obj.x0))-K*obj.C)*Ppre;
            end
            xhat=Xpre+K*(y-obj.C*Xpre);
            if Reset>0.5
                xhat=obj.x0;
                P=obj.p0;
            end
            obj.XOld=xhat;
            obj.POld=P;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

    end
end