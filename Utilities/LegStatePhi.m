classdef LegStatePhi < matlab.System
    % get yaw velocity est from the joint position feedback
    properties
        dt=0.006;
        Tstance=0.3;
    end

    properties (Access=private)
        LegStateInOld=ones(4,1);
        LegStateOutOld=ones(4,1);
        StanceFlag=ones(4,1);
        HoldFlag=ones(4,1);
        LegPhiOld=ones(4,1)*0.5;
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function [LegStateNew,LegPhi] = stepImpl(obj,LegState)
            LegStateNew=zeros(4,1);
            for i=1:1:4
                if LegState(i)>0 && LegState(i)<1 && abs(obj.LegStateInOld(i)-1)<0.01
                    LegStateNew(i)=1;
                elseif LegState(i)>0 && LegState(i)<1 && abs(obj.LegStateInOld(i)-0)<0.01
                    LegStateNew(i)=0;
                else
                    LegStateNew(i)=LegState(i);
                end
                
                if LegStateNew(i)<0.1 && obj.LegStateOutOld(i)>0.9
                    obj.StanceFlag(i)=0;
                    obj.HoldFlag(i)=0;
                elseif LegStateNew(i)>0.9 && obj.LegStateOutOld(i)<0.1
                    obj.StanceFlag(i)=1;
                end
                
                if obj.HoldFlag(i)<0.5 && LegStateNew(i)>0.5
                    obj.LegPhiOld(i)=obj.LegPhiOld(i)+obj.dt/obj.Tstance;
                elseif obj.HoldFlag(i)<0.5 && LegStateNew(i)<0.5
                    obj.LegPhiOld(i)=0;
                end
                
                if obj.LegPhiOld(i)>1
                    obj.LegPhiOld(i)=1;
                end
            end
            LegPhi=obj.LegPhiOld;

            obj.LegStateInOld=LegState;
            obj.LegStateOutOld=LegStateNew;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

    end
end