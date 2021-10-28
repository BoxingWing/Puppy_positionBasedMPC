pW=rand([3,4]);
pC=rand([3,1]);

M=[crossCap(pW(:,1)-pC),crossCap(pW(:,2)-pC),crossCap(pW(:,3)-pC),crossCap(pW(:,4)-pC); ...
                eye(3),eye(3),eye(3),eye(3)];
            
function vcap=crossCap(v)
vcap=[0,-v(3),v(2);
        v(3),0,-v(1);
       -v(2),v(1),0];
end