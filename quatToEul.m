function eul=quatToEul(quat)
%eul=quat2eul(quat);
eul1=atan2(2*(quat(1)*quat(2)+quat(3)*quat(4)),1-2*(quat(2)^2+quat(3)^2));
eul2=asin(2*(quat(1)*quat(3)-quat(2)*quat(4)));
eul3=atan2(2*(quat(1)*quat(4)+quat(2)*quat(3)),1-2*(quat(3)^2+quat(4)^2));
eul=[eul3,eul2,eul1];