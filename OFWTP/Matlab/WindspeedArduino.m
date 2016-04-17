function Wuchar1 = WindspeedArduino(WindSpeed)
Arduino = [1175 1020 890 840 800];
Winds = [6.2 7.1 8 8.35 8.85];

Wu1= round(interp1(Winds,Arduino,WindSpeed));
Wuc1 = num2str(Wu1); % Convertion of integer into char
Wuchar1 = ['(',Wuc1,')']; % Preparation for sending the data to Arduino;