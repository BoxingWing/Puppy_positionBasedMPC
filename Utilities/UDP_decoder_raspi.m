classdef UDP_decoder_raspi< matlab.System
    % Decode the data feedback from the raspberry pi.
    properties
        Inorm=diag( [ 0.0078, 0.0275, 0.0328 ] );
        m=3.5;
        hIni=0.19;
        lateral_width=0.097;
        sagetial_width=0.2108;
        roll_Off=0.037;
    end
    
    properties(Access=private)
        Inow_Old=zeros(3,3);
        U_Old=zeros(12,1);
        X_mpc_Old=zeros(13,1);
        refP_Old=zeros(13,1);
        SP_MPC_Old=zeros(12,1);
        MPC_Count_Old=0;
        LegStateMPC_Old=[1;1;1;1];
        phiSlow_Old=0;
    end
    
    methods(Access = protected)
        
        function setupImpl(obj)
            obj.Inow_Old=obj.Inorm;
            obj.U_Old=[0;0;1;0;0;1;0;0;1;0;0;1]*obj.m/4*9.8;
            
            xW=obj.lateral_width;
            yW=obj.sagetial_width;
%             obj.PendAllnorm=[0.1075,0.1075,-0.1075,-0.1075;
%                 0.0750,-0.0750,0.0750,-0.0750;
%                 0,0,0,0];
            PendAlltmp=zeros(12,1);
            PendAlltmp(1:3)=[xW;yW;0]/2;
            PendAlltmp(4:6)=[xW;-yW;0]/2;
            PendAlltmp(7:9)=[-xW;yW;0]/2;
            PendAlltmp(10:12)=[-xW;-yW;0]/2;
            PendAlltmp=PendAlltmp+[0;1;0;0;-1;0;0;1;0;0;-1;0]*obj.roll_Off;
            
            obj.SP_MPC_Old=reshape(PendAlltmp,12,1);
            obj.X_mpc_Old(3)=obj.hIni;
            obj.refP_Old(3)=obj.hIni;
            obj.MPC_Count_Old=0;
        end
        
        function [U,X_mpc,Inow,refP,SP_MPC,MPCtime,MPC_Count,EN,LegStateMPC,phiSlow] = stepImpl(obj,in)
            header=in(1);
            if abs(header-66)<0.1
                U=in(2:13);
                X_mpc=in(14:26);
                Inow=reshape(in(27:35),3,3);
                refP=in(36:48);
                SP_MPC=in(49:60);
                MPCtime=in(61);
                MPC_Count=in(62);
                LegStateMPC=in(63:66);
                phiSlow=in(67);
                EN=1;
            else
                U=obj.U_Old;
                X_mpc=obj.X_mpc_Old;
                Inow=obj.Inow_Old;
                refP=obj.refP_Old;
                SP_MPC=obj.SP_MPC_Old;
                MPC_Count=obj.MPC_Count_Old;
                LegStateMPC=obj.LegStateMPC_Old;
                phiSlow=obj.phiSlow_Old;
                MPCtime=0;
                EN=0;
            end
            obj.U_Old=U;
            obj.SP_MPC_Old=SP_MPC;
            obj.X_mpc_Old=X_mpc;
            obj.Inow_Old=Inow;
            obj.refP_Old=refP;
            obj.MPC_Count_Old=MPC_Count;
            obj.LegStateMPC_Old=LegStateMPC;
            obj.phiSlow_Old=phiSlow;
        end
        
        function [d1,d2,d3,d4,d5,d6,d7,d8,d9,d10] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'double';
            d3 = 'double';
            d4 = 'double';
            d5 = 'double';
            d6='double';
            d7= 'double';
            d8='double';
            d9='double';
            d10='double';
        end
        
        function [s1,s2,s3,s4,s5,s6,s7,s8,s9,s10] = getOutputSizeImpl(~)
            s1=[12,1];
            s2=[13,1];
            s3=[3,3];
            s4=[13,1];
            s5=[12,1];
            s6=[1,1];
            s7=[1,1];
            s8=[1,1];
            s9=[4,1];
            s10=[1,1];
        end
        
        function [f1,f2,f3,f4,f5,f6,f7,f8,f9,f10] = isOutputFixedSizeImpl(~)
            f1=true;
            f2=true;
            f3=true;
            f4=true;
            f5=true;
            f6=true;
            f7=true;
            f8=true;
            f9=true;
            f10=true;
        end
        
        function [c1,c2,c3,c4,c5,c6,c7,c8,c9,c10] = isOutputComplexImpl(~)
            c1=false;
            c2=false;
            c3=false;
            c4=false;
            c5=false;
            c6=false;
            c7=false;
            c8=false;
            c9=false;
            c10=false;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end

