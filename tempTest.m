ymRes=reshape(out.ymRes.signals.values,19,[]);
tLine=out.ymRes.time;

startT=80;
endT=88;
tmp=find(tLine>startT);
startN=tmp(1);
tmp=find(tLine>endT);
endN=tmp(1);

ymPick=ymRes(:,startN:endN);
ymStd=std(ymPick');