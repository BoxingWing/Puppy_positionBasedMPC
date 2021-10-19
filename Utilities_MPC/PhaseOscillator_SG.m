classdef PhaseOscillator_SG< matlab.System
    % single output phase oscillator
    
    properties(Nontunable)
        SampleTime = 1.4; % Sample Time
        OffsetTime = 0; % Offset Time
        TickTime = 0.1;
        startPhase=0;
        SampleTimeTypeProp (1, 1) {mustBeMember(SampleTimeTypeProp, ...
            ["Discrete","FixedInMinorStep","Controllable",...
            "Inherited","InheritedNotControllable",...
            "InheritedErrorConstant"])} = "Discrete"
    end
    
    properties(Access=private)
        tCount;
        PhaseOld;
        PauseFlag;
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
            %obj.Count = 0;
            obj.tCount=0;
            obj.PhaseOld=99;
            obj.PauseFlag=false;
        end
        
        function [phaseNow,OscStopFlag] = stepImpl(obj,omega,EN)
            phaseNow=omega*obj.SampleTime*obj.tCount+obj.startPhase;
            phaseNow=mod(phaseNow,2*pi);
            OscStopFlag=0;
            if EN>0.5 || (EN<0.5 && phaseNow>0.08)
                obj.tCount=obj.tCount+1;
                obj.PhaseOld=phaseNow;
            end
            if EN<0.5 && phaseNow<0.08
                OscStopFlag=1;
            end
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function flag = isInactivePropertyImpl(obj,prop)
            flag = false;
            switch char(obj.SampleTimeTypeProp)
                case {'Inherited', ...
                        'InheritedNotControllable', ...
                        'FixedInMinorStep'}
                    if any(strcmp(prop,{'SampleTime','OffsetTime','TickTime'}))
                        flag = true;
                    end
                case 'Discrete'
                    if any(strcmp(prop,{'TickTime'}))
                        flag = true;
                    end
                case 'Controllable'
                    if any(strcmp(prop,{'SampleTime','OffsetTime'}))
                        flag = true;
                    end
            end
        end
        
        function [d1,d2] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'double';
        end
        
        function [s1,s2] = getOutputSizeImpl(~)
            s1 = [1,1];
            s2=[1,1];
        end
        
        function [f1,f2] = isOutputFixedSizeImpl(~)
            f1 = true;
            f2 = true;
        end
        
        function [cpl1,cpl2] = isOutputComplexImpl(~)
            cpl1 = false;
            cpl2=false;
        end
        
    end
end