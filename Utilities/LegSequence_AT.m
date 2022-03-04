classdef LegSequence_AT< matlab.System
    % generate appropriate leg sequence
    properties
        tSW=0.3;
        SampleTime = 0.005; % Sample Time
        OffsetTime = 0; % Offset Time
        TickTime = 0;
        startPhase=0;
        SampleTimeTypeProp (1, 1) {mustBeMember(SampleTimeTypeProp, ...
            ["Discrete","FixedInMinorStep","Controllable",...
            "Inherited","InheritedNotControllable",...
            "InheritedErrorConstant"])} = "Discrete"
    end

    properties(Access=private)
        isLFnext=true;
        isRFnext=false;
        swCount_LF=0;
        swCount_RF=0;
        swN=1;
        xRefOld;
        LegStateOld=[1;1;1;1];
        vCoM_sw=[1;1;1]; % CoM velocity at the beginning of the last swing phase
    end

    methods(Access = protected)
        function sts = getSampleTimeImpl(obj)
            switch char(obj.SampleTimeTypeProp)
                case 'Inherited'
                    sts = createSampleTime(obj,'Type','Inherited');
                case 'InheritedNotControllable'
                    sts = createSampleTime(obj,'Type','Inherited',...
                        'AlternatePropagation','Controllable');
                case 'InheritedErrorConstant'
                    sts = createSampleTime(obj,'Type','Inherited',...
                        'ErrorOnPropagation','Constant');
                case 'FixedInMinorStep'
                    sts = createSampleTime(obj,'Type','Fixed In Minor Step');
                case 'Discrete'
                    sts = createSampleTime(obj,'Type','Discrete',...
                        'SampleTime',obj.SampleTime, ...
                        'OffsetTime',obj.OffsetTime);
                case 'Controllable'
                    sts = createSampleTime(obj,'Type','Controllable',...
                        'TickTime',obj.TickTime);
            end
        end

        function setupImpl(obj)
            obj.swN=floor(obj.tSW/obj.SampleTime);
            obj.isLFnext=true;
            obj.isRFnext=false;
            obj.swCount_LF=0;
            obj.swCount_RF=0;
            obj.LegStateOld=[1;1;1;1];
            obj.vCoM_sw=[1;1;1];
            obj.xRefOld=zeros(13,1);
        end

        function [LegState,LegPhase] = stepImpl(obj,pL_m,xFB,xRef,EN)
            % X_FB: system states from the estimator
            % pL_m: measured foot end position in leg coordinate
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

%             if vCoM_L(1)*obj.vCoM_sw(1)<0 || vCoM_L(2)*obj.vCoM_sw(2)<0
%                 obj.isLFnext=~obj.isLFnext;
%                 obj.isRFnext=~obj.isRFnext;
%             end
            
            % change the leg sequence order if CoM velocity cross zero
            if xRef(7)*obj.xRefOld(7)<0 || xRef(8)*obj.xRefOld(8)<0
                obj.isLFnext=~obj.isLFnext;
                obj.isRFnext=~obj.isRFnext;
            end
            
            if vCoM_L(1)==0 && vCoM_L(2)==0
                tRem_LFx=999;
                tRem_LFy=999;
                tRem_RFx=999;
                tRem_RFy=999;
            else
                tRem_LFx=(pL_m(1)+pL_m(10))/2/vCoM_L(1);
                tRem_LFy=(pL_m(2)+pL_m(11))/2/vCoM_L(2);
                tRem_RFx=(pL_m(4)+pL_m(7))/2/vCoM_L(1);
                tRem_RFy=(pL_m(5)+pL_m(8))/2/vCoM_L(2);
            end

            if obj.isLFnext==true
                tRem=min(tRem_LFx,tRem_LFy);
            else
                tRem=min(tRem_RFx,tRem_RFy);
            end

            LegState=obj.LegStateOld;
            if tRem<=obj.tSW/2 && sum(LegState)>3.5 && EN>0.5
                if obj.isLFnext==true
                    LegState(1)=0;
                    LegState(4)=0;
                    obj.isLFnext=~obj.isLFnext;
                    obj.isRFnext=~obj.isRFnext;
                    obj.swCount_LF=0;
                    obj.vCoM_sw=vCoM_L;
                elseif obj.isRFnext==true
                    LegState(2)=0;
                    LegState(3)=0;
                    obj.isLFnext=~obj.isLFnext;
                    obj.isRFnext=~obj.isRFnext;
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
            obj.xRefOld=xRef;
        end

        function [d1,d2] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'double';
        end

        function [s1,s2] = getOutputSizeImpl(~)
            s1 = [4,1];
            s2=[4,1];
        end

        function [f1,f2] = isOutputFixedSizeImpl(~)
            f1 = true;
            f2 = true;
        end

        function [cpl1,cpl2] = isOutputComplexImpl(~)
            cpl1 = false;
            cpl2=false;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

    end
end

function M=Rx(sita)
% 3D rotation matrix, vb=M*v:
% rotate a vector in one frame,
% or change the vector 'v' in rotated frame to 'vb' in world frame
M=[1,0,0;
    0,cos(sita),-sin(sita);
    0,sin(sita),cos(sita)];
end

function M=Ry(sita)
% 3D rotation matrix, vb=M*v:
% rotate a vector in one frame,
% or change the vector 'v' in rotated frame to 'vb' in world frame
M=[cos(sita),0,sin(sita);
    0,1,0;
    -sin(sita),0,cos(sita)];
end

function M=Rz(sita)
% 3D rotation matrix, vb=M*v:
% rotate a vector in one frame,
% or change the vector 'v' in rotated frame to 'vb' in world frame
M=[cos(sita),-sin(sita),0;
    sin(sita),cos(sita),0;
    0,0,1];
end



