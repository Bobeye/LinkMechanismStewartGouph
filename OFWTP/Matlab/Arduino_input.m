function [uchar1,u1] = Arduino_input(uContrWT1)
        XX=[-20; 0;20;40;60;80;100;120;140;160;180;200;220;225;230;235;240;245;250;255;300];
        VV=[-10000000000000;0;0.361;0.723;1.085;1.451;1.813;2.179;2.539;2.904;3.266;3.61;3.98;4.06;4.15;4.24;4.34;4.43;4.52;4.61;4.62]; %Arduino Voltage
        u1 = interp1(VV,XX,uContrWT1);
        u1 = min(u1,255); % Protection of voltage suplied for Arduino (0.7-5V)
        u1 = max(u1,0);
        u1 = round(u1); % Round the value for getting an integer in order to send it to Arduino
        uc1 = num2str(u1); % Convertion of integer into char
        uchar1 = ['(',uc1,')']; % Preparation for sending the data to Arduino
        