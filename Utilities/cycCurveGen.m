classdef cycCurveGen < matlab.System
    % Cycloid wave generator.
    properties
        Tf=0.4; % flight duration
        Ts=0.4; % stance duration
        H=40; % step height
        L=50; % stride length
        Hsd=2; % stance depth
        dt=0.005;
    end
    properties (Access=private)
        pxOld;
        dpxOld;
        pyOld;
        dpyOld;
        pxTF; % take off point of px
        pyTF; % take off point of py
        dpxTF;
        dpyTF;
        tOld;
        vx;
        xAOld;
        yAOld;
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.pxOld=0;
            obj.dpxOld=0;
            obj.pyOld=0;
            obj.dpyOld=0;
            obj.pxTF=0;
            obj.pyTF=0;
            obj.dpxTF=0;
            obj.dpyTF=0;
            obj.tOld=0;
            obj.vx=obj.L/obj.Ts;
            obj.xAOld=0;
            obj.yAOld=1;
        end
        
        function [pxOut,pyOut,pz] = stepImpl(obj,fai,vpxAdd,vpyAdd,stopInd,xADes)
            % NOTE: do NOT add vpxAdd and vpyAdd during start and stop
            px=0;
            py=0;dpy=0;
            pz=0;
            tNow=fai/2/pi*(obj.Tf+obj.Ts);
            if tNow>=obj.Ts/2 && obj.tOld<=obj.Ts/2
                obj.pxTF=obj.pxOld;
                obj.pyTF=obj.pyOld;
                obj.dpxTF=obj.dpxOld;
                obj.dpyTF=obj.dpyOld;
            end
            if tNow<=obj.Ts/2
                if abs(obj.dpxOld)<abs(obj.vx)
                    dpx=obj.dpxOld-obj.L/(obj.Ts)^2*obj.dt;
                else
                    dpx=-obj.vx;
                end
                dpx=dpx+vpxAdd;
                dpy=vpyAdd;
                if stopInd>0.5
                    aDec=2*(-obj.L/2+obj.vx*obj.Ts)/(obj.Ts)^2;
                    dpx=obj.dpxOld+aDec*obj.dt;
                end
                faiNow=-tNow/(obj.Ts/2)*pi/2+pi/2;
                pz=-obj.Hsd*sin(faiNow);
            elseif tNow>obj.Ts/2 && tNow<=obj.Ts/2+obj.Tf
                vxNew=obj.vx;
                faiNow=(tNow-obj.Ts/2)/obj.Tf*2*pi;
                if stopInd>0.5
                    rx=(-obj.pxTF+obj.Ts*vxNew)/2/pi;
                else
                    rx=(-obj.pxTF+obj.L/2+obj.Ts*vxNew)/2/pi;
                end
                ry=-obj.pyTF/2/pi;
                vxV=(rx-rx*cos(faiNow))*2*pi/obj.Tf;
                dpx=vxV-vxNew;
                dpy=(ry-ry*cos(faiNow))*2*pi/obj.Tf;
                pz=obj.H/2*(1-cos(faiNow));
            else
                if abs(obj.dpxOld)<abs(obj.vx)
                    dpx=obj.dpxOld-obj.L/(obj.Ts)^2*obj.dt;
                else
                    dpx=-obj.vx;
                end
                dpx=dpx+vpxAdd;
                dpy=vpyAdd;
                if stopInd>0.5
                    aDec=2*(-obj.L/2+obj.vx*obj.Ts)/(obj.Ts)^2;
                    dpx=obj.dpxOld+aDec*obj.dt;
                end
                tmp=tNow-obj.Ts/2-obj.Tf;
                faiNow=-tmp/(obj.Ts/2)*pi/2+pi;
                pz=-obj.Hsd*sin(faiNow);
            end
            if abs(tNow-obj.tOld)<0.0001
                    dpx=0;
                    dpy=0;
                    obj.pxTF=0;
                    obj.pyTF=0;
            end
            px=obj.pxOld+dpx*obj.dt;
            py=obj.pyOld+dpy*obj.dt;
            
            if abs(obj.xAOld-xADes)<0.004
                obj.xAOld=xADes;
            else
                obj.xAOld=obj.xAOld+sign(xADes-obj.xAOld)*1/1*0.005;
            end
            
            pxOut=px*obj.xAOld;
            pyOut=py*obj.yAOld;
            
            obj.tOld=tNow;
            obj.pxOld=px;
            obj.dpxOld=dpx;
            obj.pyOld=py;
            obj.dpyOld=dpy;
        end
        
        function flag = supportsMultipleInstanceImpl(obj)
        % Support System object in Simulink For Each subsystem
        % Do not enable For Each support if your System object allocates exclusive resources that may
        % conflict with other System objects, such as allocating file handles, memory by address, or hardware resources.
            flag = true;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
    
    methods (Static, Access=protected)
        function header = getHeaderImpl
            header=matlab.system.display.Header(mfilename('class'), ...
                'title','Cycloid curve generator',...
                   'Text',['Generate foot-end trajectory with cycloid curve in flight phase. ', ... 
               'Constant vx is assigned in stance phase.']);
        end
    end
end