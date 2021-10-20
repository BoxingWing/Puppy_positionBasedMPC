classdef message_decoder_raspi< matlab.System
    % Decode the data feedback from the raspberry pi.
    properties
        Inorm=diag( [ 0.0078, 0.0275, 0.0328 ] );
        m=3.5;
        hIni=0.19;
    end
    
    properties(Access=private)
        Inow_Old=zeros(3,3);
        U_Old=zeros(12,1);
        X_mpc_Old=zeros(13,1);
        refP_Old=zeros(13,1);
    end
    
    methods(Access = protected)
        
        function setupImpl(obj)
            obj.Inow_Old=obj.Inorm;
            obj.U_Old=[0;0;obj.m;0;0;0;0;0;0;0;0;obj.m]/2*9.8;
            obj.X_mpc_Old(3)=obj.hIni;
            obj.refP_Old(3)=obj.hIni;
        end
        
        function [U,X_mpc,Inow,refP,PCtime,EN] = stepImpl(obj,in)
            header=in(1);
            if abs(header-66)<0.1
                U=in(2:13);
                X_mpc=in(14:26);
                Inow=reshape(in(27:35),3,3);
                refP=in(36:48);
                PCtime=in(49);
                EN=1;
            else
                U=obj.U_Old;
                X_mpc=obj.X_mpc_Old;
                Inow=obj.Inow_Old;
                refP=obj.refP_Old;
                PCtime=0;
                EN=0;
            end
            obj.U_Old=U;
            obj.X_mpc_Old=X_mpc;
            obj.Inow_Old=Inow;
            obj.refP_Old=refP;
        end
        
        function [d1,d2,d3,d4,d5,d6] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'double';
            d3 = 'double';
            d4 = 'double';
            d5 = 'double';
            d6='double';
        end
        
        function [s1,s2,s3,s4,s5,s6] = getOutputSizeImpl(~)
            s1=[12,1];
            s2=[13,1];
            s3=[3,3];
            s4=[13,1];
            s5=[1,1];
            s6=[1,1];
        end
        
        function [f1,f2,f3,f4,f5,f6] = isOutputFixedSizeImpl(~)
            f1=true;
            f2=true;
            f3=true;
            f4=true;
            f5=true;
            f6=true;
        end
        
        function [c1,c2,c3,c4,c5,c6] = isOutputComplexImpl(~)
            c1=false;
            c2=false;
            c3=false;
            c4=false;
            c5=false;
            c6=false;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end

