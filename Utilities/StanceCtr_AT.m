classdef StanceCtr_AT< matlab.System
    % stance leg control module
    properties
        tSW=0.3;
        tSample=0.005;
    end

    properties(Access=private)
        isLFnext=true;
        isRFnext=false;
        swCount_LF=0;
        swCount_RF=0;
        swN=1;
        LegStateOld=[1;1;1;1];
        vCoM_sw=[1;1;1]; % CoM velocity at the beginning of the last swing phase
    end

    methods(Access = protected)

        function setupImpl(obj)
            obj.swN=floor(obj.tSW/obj.SampleTime);
            obj.isLFnext=true;
            obj.isRFnext=false;
            obj.swCount_LF=0;
            obj.swCount_RF=0;
            obj.swN=1;
            obj.LegStateOld=[1;1;1;1];
            obj.vCoM_sw=[1;1;1];
        end

        function [p_L,f_L] = stepImpl(obj,p_L_norm,xFB,EN)
            % X_FB: system states from the estimator
            % X_mpc: predicted next step's systems states from the MPC controller
            % touchInd: indicator of wether a swing leg touches the ground
            % T is the moving period

            Rrpy=Rz(xFB(6))*Ry(xFB(5))*Rx(xFB(4));
            vCoM_L=Rrpy'*xFB(7:9);

            vxDead=0.02;
            vyDead=0.02;
            if abs(vCoM_L(1))>abs(vxDead)
                vCoM_L(1)=vCoM_L(1)-sign(vCoM_L(1))*abs(vxDead);
            else
                vCoM_L(1)=0;
            end
            if abs(vCoM_L(2))>abs(vyDead)
                vCoM_L(2)=vCoM_L(2)-sign(vCoM_L(2))*abs(vyDead);
            else
                vCoM_L(2)=0;
            end

            if vCoM_L(1)*obj.vCoM_sw(1)<0 || vCoM_L(2)*obj.vCoM_sw(2)<0
                obj.isLFnext=~obj.isLFnext;
                obj.isRFnext=~obj.isRFnext;
            end

            tRem_LFx=(p_L(1)+p_L(10))/2/vCoM_L(1);
            tRem_LFy=(p_L(2)+p_L(11))/2/vCoM_L(2);
            tRem_RFx=(p_L(4)+p_L(7))/2/vCoM_L(1);
            tRem_RFy=(p_L(5)+p_L(8))/2/vCoM_L(2);

            if obj.isLFnext==true
                tRem=min(tRem_LFx,tRem_LFy);
            else
                tRem=min(tRem_RFx,tRem_RFy);
            end

            LegState=obj.LegStateOld;
            if tRem<0 && sum(LegState)>3.5 && EN>0.5
                if obj.isLFnext==true
                    LegState(1)=0;
                    LegState(4)=0;
                    obj.isLFnext=false;
                    obj.isRFnext=true;
                    obj.swCount_LF=0;
                    obj.vCoM_sw=vCoM_L;
                elseif obj.isRFnext==true
                    LegState(2)=0;
                    LegState(3)=0;
                    obj.isLFnext=true;
                    obj.isRFnext=false;
                    obj.swCount_RF=0;
                    obj.vCoM_sw=vCoM_L;
                end
            end

            LegPhase=ones(4,1);
            if LegState(1)<0.5
                obj.swCount_LF=obj.swCount_LF+1;
                LegPhase(1)=obj.swCount_LF/obj.swN;
                LegPhase(4)=LegPhase(1);
                if obj.swCount_LF>obj.swN
                    %                     obj.swCount_LF=0;
                    LegState(1)=1;
                    LegState(4)=1;
                end
            end
            if LegState(2)<0.5
                obj.swCount_RF=obj.swCount_RF+1;
                LegPhase(2)=obj.swCount_RF/obj.swN;
                LegPhase(3)=LegPhase(2);
                if obj.swCount_RF>obj.swN
                    %                     obj.swCount_RF=0;
                    LegState(2)=1;
                    LegState(3)=1;
                end
            end

            %%% data update
            obj.LegStateOld=LegState;

        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

    end
end




