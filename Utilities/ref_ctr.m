classdef ref_ctr< matlab.System
    % regulate the commanded ref vx, vy and wz.
    properties
        vxMax=0.4;
        vyMax=0.4;
        wzMax=0.2;
    end
    
    properties(Access=private)
        vx_Old=0;
        vy_Old=0;
        wz_Old=0;
    end
    
    methods(Access = protected)
        
        function setupImpl(obj)
            
        end
        
        function [vx,vy,wz] = stepImpl(obj,timeIn,disable,vxPercent,vyPercent,wzPercent)
            if abs(disable-1)<0.1
                vx=0;
                vy=0;
                wz=0;
            else
                vx=vxPercent*obj.vxMax;
                vy=vyPercent*obj.vyMax;
                wz=wzPercent*obj.wzMax;
            end
            if abs(vx)>obj.vxMax
                vx=sign(vx)*obj.vxMax;
            end
            if abs(vy)>obj.vyMax
                vy=sign(vy)*obj.vyMax;
            end
            if abs(wz)>obj.wzMax
                wz=sign(wz)*obj.wzMax;
            end
            obj.vx_Old=vx;
            obj.vy_Old=vy;
            obj.wz_Old=wz;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end

