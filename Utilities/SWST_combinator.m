classdef SWST_combinator< matlab.System
    % combine the swing and stance leg control commands
    properties
        
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

        function [fL_res,pL_bas,pL_LF,fw_res] = stepImpl(obj,LegState,pL_st,pL_sw,xFB,xRef,fL,fW)
            fL=reshape(fL,3,4);
            fW=reshape(fW,3,4);
            Rdef=Rz(xRef(6))*Ry(xRef(5))*Rx(xRef(4));
            fW2L=Rdef'*fW;
            pL_st=reshape(pL_st,3,4);
            pL_sw=reshape(pL_sw,3,4);
            fL_res=zeros(3,4);
            pL_bas=zeros(3,4);
            pL_LF=zeros(3,4);
            for i=1:1:4
                if LegState(i)>0.5
                    pL_bas(:,i)=pL_st(:,i);
                    fL_res(:,i)=fL(:,i)+fW2L(:,i);
                    pL_LF(:,i)=pL_bas(:,i);
                else
                    pL_bas(:,i)=pL_sw(:,i);
                    pL_LF(:,i)=obj.pL_LF_Old(:,i);
                end
            end
            
            Rnow=Rz(xFB(6))*Ry(xFB(5))*Rx(xFB(4));
            fw_res=Rnow*fL_res;
            obj.pL_LF_Old=pL_LF;
            obj.LegStateOld=LegState;

            fL_res=reshape(fL_res,12,1);
            pL_bas=reshape(pL_bas,12,1);
            pL_LF=reshape(pL_LF,12,1);
            fw_res=reshape(fw_res,12,1);
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

    end
end
%% subfunction
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