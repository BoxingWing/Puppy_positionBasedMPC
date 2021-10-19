classdef message_decoder_PC< matlab.System
    % Decode the data feedback from the raspberry pi.
    properties
        lateral_width=0.097;
        sagetial_width=0.2108;
        roll_Off=0.037;
        hIni=0.19;
    end
    
    properties(Access=private)
        pCoM_Old=zeros(3,1);
        vCoM_Old=zeros(3,1);
        RPYnew_Old=zeros(3,1);
        OmegaW_Old=zeros(3,1);
        SPLeg_Old=zeros(4,1);
        SP_Old=zeros(12,1);
        surVN_Old=zeros(3,1);
        surV1_Old=zeros(3,1);
        surV2_Old=zeros(3,1);
        surP_Old=zeros(3,1);
        headG_Old=zeros(3,1);
        vxPercent_Old=0;
        vyPercent_Old=0;
        wzPercent_Old=0;
        phi_Old=0;
        estDis_Old=zeros(6,1);
    end
    
    methods(Access = protected)
        
        function setupImpl(obj)
            obj.pCoM_Old=[0;0;obj.hIni];
            xW=obj.lateral_width;
            yW=obj.sagetial_width;
            obj.SP_Old=zeros(12,1);
            obj.SP_Old(1:3)=[xW;yW;0]/2;
            obj.SP_Old(4:6)=[xW;-yW;0]/2;
            obj.SP_Old(7:9)=[-xW;yW;0]/2;
            obj.SP_Old(10:12)=[-xW;-yW;0]/2;
            obj.SP_Old=obj.SP_Old+[0;1;0;0;-1;0;0;1;0;0;-1;0]*obj.roll_Off;
            obj.SPLeg_Old=[1;0;0;1];
        end
        
        function [pCoM,vCoM,RPYnew,OmegaW,SPLeg,SP,surVN,surV1,surV2,surP,headG,mpcStop,phi,vxPercent,vyPercent,wzPercent,estDis] = stepImpl(obj,in)
            header=in(1);
            if abs(header-66)<0.1
                pCoM=in(2:4);
                vCoM=in(5:7);
                RPYnew=in(8:10);
                OmegaW=in(11:13);
                SPLeg=in(14:17);
                SP=in(18:29);
                surVN=in(30:32);
                surV1=in(33:35);
                surV2=in(36:38);
                surP=in(39:41);
                headG=in(42:44);
                mpcStop=in(45);
                phi=in(46);
                vxPercent=in(47);
                vyPercent=in(48);
                wzPercent=in(49);
                estDis=in(50:55);
            else
                pCoM=obj.pCoM_Old;
                vCoM=obj.vCoM_Old;
                RPYnew=obj.RPYnew_Old;
                OmegaW=obj.OmegaW_Old;
                SPLeg=obj.SPLeg_Old;
                SP=obj.SP_Old;
                surVN=obj.surVN_Old;
                surV1=obj.surV1_Old;
                surV2=obj.surV2_Old;
                surP=obj.surP_Old;
                headG=obj.headG_Old;
                mpcStop=1;
                phi=obj.phi_Old;
                vxPercent=obj.vxPercent_Old;
                vyPercent=obj.vyPercent_Old;
                wzPercent=obj.wzPercent_Old;
                estDis=obj.estDis_Old;
            end
            obj.pCoM_Old=pCoM;
            obj.vCoM_Old=vCoM;
            obj.RPYnew_Old=RPYnew;
            obj.OmegaW_Old=OmegaW;
            obj.SPLeg_Old=SPLeg;
            obj.SP_Old=SP;
            obj.surVN_Old=surVN(1:3);
            obj.surV1_Old=surV1(1:3);
            obj.surV2_Old=surV2(1:3);
            obj.surP_Old=surP(1:3);
            obj.headG_Old=headG(1:3);
            obj.phi_Old=phi;
            obj.estDis_Old=estDis;
        end
        
        function [d1,d2,d3,d4,d5,d6,d7,d8,d9,d10,d11,d12,d13,d14,d15,d16,d17] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'double';
            d3 = 'double';
            d4 = 'double';
            d5 = 'double';
            d6 = 'double';
            d7 = 'double';
            d8 = 'double';
            d9 = 'double';
            d10 = 'double';
            d11 = 'double';
            d12='double';
            d13='double';
            d14='double';
            d15='double';
            d16='double';
            d17='double';
        end
        
        function [s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15,s16,s17] = getOutputSizeImpl(~)
            s1 = [3,1];
            s2=[3,1];
            s3=[3,1];
            s4=[3,1];
            s5=[4,1];
            s6=[12,1];
            s7=[3,1];
            s8=[3,1];
            s9=[3,1];
            s10=[3,1];
            s11=[3,1];
            s12=[1,1];
            s13=[1,1];
            s14=[1,1];
            s15=[1,1];
            s16=[1,1];
            s17=[6,1];
        end
        
        function [f1,f2,f3,f4,f5,f6,f7,f8,f9,f10,f11,f12,f13,f14,f15,f16,f17] = isOutputFixedSizeImpl(~)
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
            f11=true;
            f12=true;
            f13=true;
            f14=true;
            f15=true;
            f16=true;
            f17=true;
        end
        
        function [c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,c16,c17] = isOutputComplexImpl(~)
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
            c11=false;
            c12=false;
            c13=false;
            c14=false;
            c15=false;
            c16=false;
            c17=false;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end

