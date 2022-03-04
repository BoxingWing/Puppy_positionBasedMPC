classdef groundAdaptionP1 < matlab.System
    % extract last step foot position in the world coordinate
    properties
        lateral_width=0.097;
        sagetial_width=0.2108;
        roll_Off=0.037;
    end
    properties (Access=private)
        pW_Old=zeros(3,4);
        LegState_Old=ones(4,1);
        pW_LS;
        pWnorm;
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            yW=obj.lateral_width;
            xW=obj.sagetial_width;
            %             obj.PendAllnorm=[0.1075,0.1075,-0.1075,-0.1075;
            %                 0.0750,-0.0750,0.0750,-0.0750;
            %                 0,0,0,0];
            PendAlltmp=zeros(12,1);
            PendAlltmp(1:3)=[xW;yW;0]/2;
            PendAlltmp(4:6)=[xW;-yW;0]/2;
            PendAlltmp(7:9)=[-xW;yW;0]/2;
            PendAlltmp(10:12)=[-xW;-yW;0]/2;
            PendAlltmp=PendAlltmp+[0;1;0;0;-1;0;0;1;0;0;-1;0]*obj.roll_Off;
            obj.pWnorm=reshape(PendAlltmp,3,4);
            obj.pW_Old=reshape(PendAlltmp,3,4);
            obj.pW_LS=reshape(PendAlltmp,3,4);
        end
        
        function [pW,pW_LS] = stepImpl(obj,LegState,SP)
            %pW=round(SP*10^5)/(10^5); % round to 0.1 mm
            pW=SP;
            pW_LS=obj.pW_LS;
            
            for i=1:1:4
                if LegState(i)>0.5 && obj.LegState_Old(i)<0.5
                    pW_LS(:,i)=pW(:,i);
                end
            end
            
%             if Disable>0.5
%                 pW=obj.pWnorm;
%                 pW_LS=obj.pWnorm;
%             end
            
            obj.pW_LS=pW_LS;
            obj.LegState_Old=LegState;
            obj.pW_Old=pW;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end



