classdef JoystickRead < realtime.internal.SourceSampleTime ...
        & coder.ExternalDependency ...
        & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
    %
    % to read the joystick input of a raspberry pi. The device must be mounted to '/dev/input/js0'
    % 
    % This template includes most, but not all, possible properties,
    % attributes, and methods that you can implement for a System object in
    % Simulink.
    %
    % NOTE: When renaming the class name Source, the file name and
    % constructor name must be updated to use the class name.
    %
    
    % Copyright 2016-2018 The MathWorks, Inc.
    %#codegen
    %#ok<*EMCA>
    
    properties
        % Public, tunable properties.
    end
    
    properties (Nontunable)
        % Public, non-tunable properties.
    end
    
    properties (Access = private)
        % Pre-computed constants.
    end
    
    methods
        % Constructor
        function obj = JoystickRead(varargin)
            % Support name-value pair arguments when constructing the object.
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods (Access=protected)
        function setupImpl(obj) %#ok<MANU>
            if isempty(coder.target)
                % Place simulation setup code here
            else
                % Call C-function implementing device initialization
                coder.cinclude('joystick_raspi.h');
                coder.ceval('joystickSetup');
            end
        end
        
        function [EventNum,value,number,buttonState,axisState] = stepImpl(obj)   %#ok<MANU>
            EventNum=int16(0);
            value = int16(ones(64,1));
            number = int16(ones(64,1));
            buttonState=uint8(0);
            axisState=uint8(0);
            if isempty(coder.target)
                % Place simulation output code here
            else
                % Call C-function implementing device output
                %y = coder.ceval('source_output');
                coder.ceval('eventRead');
                EventNum=coder.ceval('returnJSInfo',coder.wref(value),coder.wref(number));
                buttonState=coder.ceval('returnButtonState');
                axisState=coder.ceval('returnAxisState');
            end
        end
        
        function releaseImpl(obj) %#ok<MANU>
            if isempty(coder.target)
                % Place simulation termination code here
            else
                % Call C-function implementing device termination
                %coder.ceval('source_terminate');
            end
        end

        %% Define output properties
        function num = getNumInputsImpl(~)
            num = 0;
        end
        
        function num = getNumOutputsImpl(~)
            num = 5;
        end
        
        function flag = isOutputSizeLockedImpl(~,~)
            flag = true;
        end
        
        function varargout = isOutputFixedSizeImpl(~,~)
            varargout{1} = true;
            varargout{2} = true;
            varargout{3} = true;
            varargout{4} = true;
            varargout{5} = true;
        end
        
        function flag = isOutputComplexityLockedImpl(~,~)
            flag = true;
        end
        
        function varargout = isOutputComplexImpl(~)
            varargout{1} = false;
            varargout{2} = false;
            varargout{3} = false;
            varargout{4} = false;
            varargout{5} = false;
        end
        
        function varargout = getOutputSizeImpl(~)
            varargout{1} = 1;
            varargout{2} = [64,1];
            varargout{3} = [64,1];
            varargout{4} = 1;
            varargout{5} = 1;
        end
        
        function varargout = getOutputDataTypeImpl(~)
            varargout{1} = 'int16';
            varargout{2} = 'int16';
            varargout{3} = 'int16';
            varargout{4} = 'uint8';
            varargout{5} = 'uint8';
        end
        
        function icon = getIconImpl(~)
            % Define a string as the icon for the System block in Simulink.
            iconStr=sprintf('Bluetooth\n Joystick Reader');
            icon = iconStr;
        end    
    end
    
    methods (Static, Access=protected)
        function simMode = getSimulateUsingImpl(~)
            simMode = 'Code generation';
        end
        
        function isVisible = showSimulateUsingImpl
            isVisible = false;
        end
        
        function header = getHeaderImpl
            title=sprintf('Bluetooth Joystick Reader');
            header=matlab.system.display.Header(mfilename('class'), ...
                   'Title',title,...
                   'Text','Read control commands from a bluetooth joystick, which must be mounted to "/dev/input/js0".');
        end
    end
    
    methods (Static)
        function name = getDescriptiveName()
            name = 'JoyStick Reader';
        end
        
        function b = isSupportedContext(context)
            b = context.isCodeGenTarget('rtw');
        end
        
        function updateBuildInfo(buildInfo, context)
            if context.isCodeGenTarget('rtw')
                % Update buildInfo
                srcDir = fullfile(fileparts(mfilename('fullpath')),'src'); 
                includeDir = fullfile(fileparts(mfilename('fullpath')),'include');
                addIncludePaths(buildInfo,includeDir);
                addSourceFiles(buildInfo,'joystick_raspi.c',srcDir);
                % Use the following API's to add include files, sources and
                % linker flags
                %addIncludeFiles(buildInfo,'source.h',includeDir);
                %addSourceFiles(buildInfo,'source.c',srcDir);
                %addLinkFlags(buildInfo,{'-lSource'});
                %addLinkObjects(buildInfo,'sourcelib.a',srcDir);
                %addCompileFlags(buildInfo,{'-D_DEBUG=1'});
                %addDefines(buildInfo,'MY_DEFINE_1')
            end
        end
    end
end
