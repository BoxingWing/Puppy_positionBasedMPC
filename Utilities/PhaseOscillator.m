classdef PhaseOscillator< matlab.System
    % Counts Hits and Time. When EN==false, output will stop until
    % phaseNow(1) becomes 0.
    
    properties(Nontunable)
        SampleTime = 0.005; % Sample Time
        OffsetTime = 0; % Offset Time
        TickTime = 0.1;
        SampleTimeTypeProp (1, 1) {mustBeMember(SampleTimeTypeProp, ...
            ["Discrete","FixedInMinorStep","Controllable",...
            "Inherited","InheritedNotControllable",...
            "InheritedErrorConstant"])} = "Discrete"
    end
    
    properties(DiscreteState)
        tCount;
        PhaseOld;
        runFlag;
        stopIndFlag; % stop flag for other functions to use
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
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.PhaseOld=[0;0;0;0];
            obj.runFlag=false;
            obj.stopIndFlag=false;
        end
        
        function [phaseNow,stopIndFlag] = stepImpl(obj,omega,phaseLag,stopIndPhase,stopPhase,EN)
            stopIndFlag=obj.stopIndFlag;
            phaseNow=obj.PhaseOld;
             if EN>0.5 && obj.runFlag==false
                obj.runFlag=true;
                obj.PhaseOld=phaseLag;
                obj.stopIndFlag=false;
            end
            if obj.runFlag==true
                for i=1:1:4
                    phaseNow(i)=omega*obj.SampleTime+obj.PhaseOld(i);
                    if phaseNow(i)>=2*pi
                        phaseNow(i)=0;
                    end
                end
            end
            if EN<0.5 && phaseNow(1)>=stopPhase && obj.PhaseOld(1)<stopPhase && obj.stopIndFlag==true
                obj.runFlag=false;
            end
            if EN<0.5 && phaseNow(1)>=stopIndPhase && obj.PhaseOld(1)<stopIndPhase
                obj.stopIndFlag=true;
            end
            obj.PhaseOld=phaseNow;
        end
        
        function setupImpl(obj)
            %obj.Count = 0;
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
    end
    methods (Static, Access=protected)
        function header = getHeaderImpl
            header=matlab.system.display.Header(mfilename('class'), ...
                   'Title','Phase Oscillator',...
                   'Text',['Counts Hits and Time.', ...
                   ' When EN==false, StopIndFlag will firstly be triggered when the phase(1) crosses the stopIndPhase, ',...
                   'then all phase will grow until phase(1) reaches the stopPhase. ', ...
                   'Note that as long as phase(1) reaches the stopPhase, all phase will be reseted to their initial values']);
        end
    end
end