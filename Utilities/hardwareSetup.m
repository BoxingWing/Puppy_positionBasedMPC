% to reset the low-level stm32 board
mypi2=raspi;
configurePin(mypi2,21,'DigitalOutput');
writeDigitalPin(mypi2,21,0);
pause(2); % pause for 1 s
writeDigitalPin(mypi2,21,1);
pause(2); % pause for 1 s
clear mypi2;