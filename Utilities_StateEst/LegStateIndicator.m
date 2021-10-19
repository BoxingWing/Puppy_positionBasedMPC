classdef LegStateIndicator < matlab.System
    %  SP: [3,4], the foot-end position in the world coordinate; SPLeg: [4,1], on or off gournd indicator
    properties (Access=private)
        SWOld=[0;0;0;0];
        SPOld=zeros(3,4);
        count=0;
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants

        end
        
        function [SPLeg,SP] = stepImpl(obj,SW,pArray_W0,pArray_W,reset)
            % SW: [4,1], indicates the touch state of four legs
            % pArray_W:[3,4], the foot-end position in the world coordinate
            % SP: [3,4], the foot-end position in the world coordinate
            % SPLeg: [4,1], on or off gournd indicator
            SPLeg=zeros(4,1);
            SP=zeros(3,4);
            if obj.count<0.5 || reset>0.5
                obj.SPOld=reshape(pArray_W0,3,4);
                obj.count=obj.count+1;
            end
            pArray_W=reshape(pArray_W,3,4);
            for i=1:1:4
                if SW(i)>0.5
                    SPLeg(i)=1;
                    if obj.SWOld(i)>0.5
                        SP(:,i)=obj.SPOld(:,i);
                    else
                        SP(:,i)=pArray_W(:,i);
                    end
%                     SP(:,i)=pArray_W(:,i);
                else
                    SPLeg(i)=0;
                    SP(:,i)=pArray_W(:,i);
                end
            end
            if reset>0.5
                SP=obj.SPOld;
            end
            obj.SWOld=SPLeg;
            obj.SPOld=SP;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

    end
end
function M=Rx(sita)
% 3D rotation matrix, from world to body
M=[1,0,0;
    0,cos(sita),-sin(sita);
    0,sin(sita),cos(sita)];
end

function M=Ry(sita)
% 3D rotation matrix, from world to body
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