classdef LegStateIndicator_Compound < matlab.System
    %  SP: [3,4], the foot-end position in the world coordinate; SPLeg: [4,1], on or off gournd indicator
    properties (Access=private)
        SWOld=[0;0;0;0];
        SPOld=zeros(3,4);
        SWStore=zeros(4,2);
        SP_td=zeros(3,4);
        LegStateOld=ones(4,1);
        count=0;
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants

        end
        
        function [SPLeg,SP,SP_Pse] = stepImpl(obj,SW,pArray_W0,pArray_W,reset,LegState)
            % SW: [4,1], indicates the touch state of four legs
            % pArray_W:[3,4], the foot-end position in the world coordinate
            % SP: [3,4], the foot-end position in the world coordinate
            % SPLeg: [4,1], on or off gournd indicator
            % SP_Pse: [3,4], foot-end positions in the world coordinate with strict contact costraint

            if obj.count<0.5 || reset>0.5
                obj.SPOld=reshape(pArray_W0,3,4);
                obj.SP_td=reshape(pArray_W0,3,4);
                obj.LegStateOld=LegState;
                obj.count=obj.count+1;
            end
            SP=reshape(pArray_W,3,4);
            
            SWnow=zeros(4,1);
            for i=1:1:4
                if SW(i)>0.5 && sum(obj.SWStore(i,:))>length(obj.SWStore(i,:))-0.5
                    SWnow(i)=1;
                elseif SW(i)<0.5 && sum(obj.SWStore(i,:))<0.5
                    SWnow(i)=0;
                else
                    SWnow(i)=obj.SWOld(i);
                end
            end
            obj.SWStore(:,1:end-1)=obj.SWStore(:,2:end);
            obj.SWStore(:,end)=SW;
            %SWnow=SW; % disable tacktile switch deshake
            SPLeg=SWnow;
            
            SP_Pse=SP;
            for i=1:1:4
                if LegState(i)>0.5 && obj.LegStateOld(i)<0.5
                    obj.SP_td(:,i)=SP(:,i);
                end
                if LegState(i)>0.5
                    SP_Pse(:,i)=obj.SP_td(:,i);
                end
            end


            if reset>0.5
                SP=obj.SPOld;
            end
            obj.LegStateOld=LegState;
            obj.SWOld=SWnow;
            obj.SPOld=SP;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

    end
end
% function M=Rx(sita)
% % 3D rotation matrix, from world to body
% M=[1,0,0;
%     0,cos(sita),-sin(sita);
%     0,sin(sita),cos(sita)];
% end
% 
% function M=Ry(sita)
% % 3D rotation matrix, from world to body
% M=[cos(sita),0,sin(sita);
%     0,1,0;
%     -sin(sita),0,cos(sita)];
% end
% 
% function M=Rz(sita)
% % 3D rotation matrix, vb=M*v: 
% % rotate a vector in one frame,
% % or change the vector 'v' in rotated frame to 'vb' in world frame
% M=[cos(sita),-sin(sita),0;
%     sin(sita),cos(sita),0;
%     0,0,1];
% end