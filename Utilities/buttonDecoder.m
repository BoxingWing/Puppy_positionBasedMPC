classdef buttonDecoder < matlab.System
    % Button decoder for 8bitdo zero2 gamepad.
    properties (Access=private)
        XOld;
        YOld;
        AOld;
        BOld;
        upOld;
        downOld;
        leftOld;
        rightOld;
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
        end
        
        function [X,Y,A,B,up,down,left,right] = stepImpl(obj,value,number)
            X=obj.XOld;
            Y=obj.YOld;
            A=obj.AOld;
            B=obj.BOld;
            left=obj.leftOld;
            right=obj.rightOld;
            up=obj.upOld;
            down=obj.downOld;
           switch number
               case 3
                   if value==1
                        X=mod(obj.XOld+1,2);
                   end
               case 4
                   if value==1
                        Y=mod(obj.YOld+1,2);
                   end
               case 0
                   if value==1
                        A=mod(obj.AOld+1,2);
                   elseif value<-1000
                       left=mod(obj.leftOld+1,2);
                   elseif value>1000
                       right=mod(obj.rightOld+1,2);
                   end
               case 1
                   if value==1
                        B=mod(obj.BOld+1,2);
                   elseif value<-1000
                       up=mod(obj.upOld+1,2);
                   elseif value>1000
                       down=mod(obj.downOld+1,2);
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
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
    
    methods (Static, Access=protected)
        function header = getHeaderImpl
            header=matlab.system.display.Header(mfilename('class'), ...
                'title','Button decoder',...
                   'Text',['Output the self-lock button state,', ...
                   ' i.e. every button state is hold until the corresponding button is pressed again.']);
        end
    end
end