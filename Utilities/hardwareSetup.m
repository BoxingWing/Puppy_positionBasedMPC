% to reset the low-level stm32 board
mypi2=raspi('192.168.3.15');
configurePin(mypi2,21,'DigitalOutput');
writeDigitalPin(mypi2,21,0);
pause(2); % pause for 1 s
writeDigitalPin(mypi2,21,1);
pause(2); % pause for 1 s
clear mypi2;