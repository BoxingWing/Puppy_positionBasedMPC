classdef SWST_combinator< matlab.System
    % combine the swing and stance leg control commands
    properties
        r0=0.19;
    end

    properties(Access=private)
        LegStateOld;
        pL_LF_Old;
    end

    methods(Access = protected)

        function setupImpl(obj)
           obj.LegStateOld=[1;1;1;1];
           obj.pL_LF_Old=[0;0;-0.19]*[1,1,1,1];
        end

        function [fL_res,pL_bas,pL_LF] = stepImpl(obj,LegState,pL_st,pL_sw,fL1,fL2)
            fL1=reshape(fL1,3,4);
            fL2=reshape(fL2,3,4);
            pL_st=reshape(pL_st,3,4);
            pL_sw=reshape(pL_sw,3,4);
            fL_res=zeros(3,4);
            pL_bas=zeros(3,4);
            pL_LF=zeros(3,4);
            for i=1:1:4
                if LegState(i)>0.5
                    pL_bas(:,i)=pL_st(:,i);
                    fL_res(:,i)=fL1(:,i)+fL2(:,i);
                    pL_LF(:,i)=pL_bas(:,i);
                else
                    pL_bas(:,i)=pL_sw(:,i);
                    pL_LF(:,i)=obj.pL_LF_Old(:,i);
                end
            end

            obj.pL_LF_Old=pL_LF;
            obj.LegStateOld=LegState;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

    end
end
