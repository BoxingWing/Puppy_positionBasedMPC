classdef Uregulation_QP< matlab.System
    % generate online state space model
    % x=[r,theta,dr,omega,g];
    % u=[f1,f2,f3,f4];
    % y=[r,theta,dr,omega,g]
    properties
        miu=0.5;
        maxFz=50;
        minFz=6;
    end
    
    properties(Access=private)
        
    end
    
    methods(Access = protected)
        
        function setupImpl(obj)
            
        end
        
        function [Unew,exitflag] = stepImpl(obj,U1,U2,surVN,surVH,surVL,LegState)
            % surVN: norm vector of the surface
            % surVH: heading vector along the surface
            % surVL: lateral vector along the surface
            Uadd=U1+U2;
            
            % friction cone
            A1=zeros(8,12);
            b1=zeros(8,1);
            for i=1:1:4
                A1(i,3*i-2:3*i)=-obj.miu*surVN+surVH;
                A1(i+4,3*i-2:3*i)=-obj.miu*surVN-surVH;
            end
            A2=zeros(8,12);
            b2=zeros(8,1);
            for i=1:1:4
                A2(i,3*i-2:3*i)=-obj.miu*surVN+surVL;
                A2(i+4,3*i-2:3*i)=-obj.miu*surVN-surVL;
            end
            % support force greater than zero, gait define
            A3=zeros(8,12);
            b3=zeros(8,1);
            for i=1:1:4
                A3(i,3*i-2:3*i)=surVN;
                A3(i+4,3*i-2:3*i)=-surVN;
                b3(i)=obj.maxFz;
                b3(i+4)=obj.minFz;
            end
            for i=1:1:4
                if LegState(i)<0.5
                    b3(i)=0;
                    b3(i+4)=0;
                end
            end
            
            A=[A1;A2;A3];
            b=[b1;b2;b3];
            
            bdelta=b-A*Uadd;
            
            H=diag(ones(12,1));
            f=zeros(12,1);
            
            opts = optimoptions('quadprog','Algorithm','active-set','MaxIterations',3);
            [Udelta,~,exitflag] = quadprog(H,f,A,bdelta,[],[],[],[],zeros(12,1),opts);
            if exitflag<0
                Udelta=zeros(12,1);
            end
            Unew=Uadd+Udelta;
        end
        
        function [d1,d2] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'double';
        end
        
        function [s1,s2] = getOutputSizeImpl(~)
            s1=[12,1];
            s2=[1,1];
        end
        
        function [f1,f2] = isOutputFixedSizeImpl(~)
            f1=true;
            f2=true;
        end
        
        function [c1,c2] = isOutputComplexImpl(~)
            c1=false;
            c2=false;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end
