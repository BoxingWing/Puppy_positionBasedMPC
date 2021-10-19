classdef buttonDecoder_2 < matlab.System
    % Button decoder for 8bitdo sn30 pro gamepad.
    % the value of each button will be held to 1 from pressed to released
    % for axis:
    %  up and right are positive
    %  output value is nomalized to range of 1
    properties (Access=private)
        XOld;
        YOld;
        AOld;
        BOld;
        upOld;
        downOld;
        leftOld;
        rightOld;
        LeftAxis; % first value for up and down diretcion, the second one for left and right direction
        RightAxis;
        count; % to avoid wrong first read data
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.XOld=0;
            obj.YOld=0;
            obj.AOld=0;
            obj.BOld=0;
            obj.upOld=0;
            obj.downOld=0;
            obj.leftOld=0;
            obj.rightOld=0;
            obj.count=0;
            obj.LeftAxis=[0;0];
            obj.RightAxis=[0;0];
        end
        
        function [X,Y,A,B,up,down,left,right,LAxis,RAxis] = stepImpl(obj,Eventnum,value,number)
            value=int16(value);
            number=int16(number);
            X=obj.XOld;
            Y=obj.YOld;
            A=obj.AOld;
            B=obj.BOld;
            left=obj.leftOld;
            right=obj.rightOld;
            up=obj.upOld;
            down=obj.downOld;
            LAxis=obj.LeftAxis;
            RAxis=obj.RightAxis;
            if obj.count>=10 && Eventnum>0
                for i=1:1:Eventnum+1
                    switch number(i)
                        case 0
                            if value(i)==1
                                A=1;
                            elseif value(i)==0
                                A=0;
                            end
                            LAxis(2)=double(value(i))/32767;
                        case 1
                            if value(i)==1
                                B=1;
                            elseif value(i)==0
                                B=0;
                            end
                            LAxis(1)=-double(value(i))/32767;
                        case 2
                            RAxis(2)=double(value(i))/32767;
                        case 3
                            if value(i)==1
                                X=1;
                            elseif value(i)==0
                                X=0;
                            end
                            RAxis(1)=-double(value(i))/32767;
                        case 4
                            if value(i)==1
                                Y=1;
                            elseif value(i)==0
                                Y=0;
                                left=0;
                                right=0;
                            elseif value(i)==-32767
                                left=1;
                            elseif value(i)==32767
                                right=1;
                            end
                        case 5
                            if value(i)==32767
                                down=1;
                            elseif value(i)==-32767
                                up=1;
                            elseif value(i)==0
                                down=0;
                                up=0;
                            end
                    end
                end
            end
            obj.XOld=X;
            obj.YOld=Y;
            obj.AOld=A;
            obj.BOld=B;
            obj.upOld=up;
            obj.downOld=down;
            obj.leftOld=left;
            obj.rightOld=right;
            obj.LeftAxis=LAxis;
            obj.RightAxis=RAxis;
            obj.count=obj.count+1;
            if obj.count>=60
                obj.count=60;
            end
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
    
    methods (Static, Access=protected)
        function header = getHeaderImpl
            header=matlab.system.display.Header(mfilename('class'), ...
                'title','Button decoder',...
                'Text',['For 8bitdo SN30 pro, output the latched button state,', ...
                ' i.e. every button state is hold until released.']);
        end
    end
end