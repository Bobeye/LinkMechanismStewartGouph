%% Lego wind farm, ----------
    COM_CloseNXT all
    close all
    clear all
    clc
    
%% Initialize COM port of Arduino   
    comPort = 'COM9'; % Wind Tunnel
    s = serial(comPort);
    set(s,'DataBits',8);
    set(s,'StopBits',1);
    set(s,'BaudRate',9600);
    set(s,'Parity', 'none');
    fopen(s);

    comPort1 = 'COM4'; % Wind Turbines
    sf = serial(comPort1);
    set(sf,'DataBits',8);
    set(sf,'StopBits',1);
    set(sf,'BaudRate',9600);
    set(sf,'Parity', 'none');
    fopen(sf);
%% USB PORTS 
% Initialization of NXT's
   NXT(1) = COM_OpenNXTEx('USB', '00165318BA59'); % NXT1 - Turbine 1   %0016531453A1  00165318BA59
  %~~~~~~~~~~~~~~~~~~~~~~~~
    nTurb = 1;   % Number of turbines
    nAdd = 0;    % Number of additional NXTs
%% Experiment Configuration
igSensorNumber = 7; % igSensorNumber = number of sensor for current
vgSensorNumber = 2; % vgSensorNumber = number of sensor for voltage
numGen = 11;         % numGen = number of generator
directMov =1;       % directMov = 1 for clockwise, = 2 for counter clockwise
vertHoriz =2;       % vertHoriz = 1 for vertical shaft, = 2 for horizontal shaft
radRot = 0.145;     % radRot = rotor radius (m)
areaRot = radRot^2*pi;  % areaRot = rotor area (m2). Classical is = pi*radRot^2
%% STATUS 
% Indicates if the sensor is working correctly
    for n=1:nTurb+nAdd  % NXT Output port
        statusSENSOR_1 = NXT_SetInputMode(SENSOR_1, 'LOWSPEED_9V', 'RAWMODE','dontreply', NXT(n)); 
        statusSENSOR_2 = NXT_SetInputMode(SENSOR_2, 'LOWSPEED_9V', 'RAWMODE','dontreply', NXT(n));
        statusSENSOR_3 = NXT_SetInputMode(SENSOR_3, 'LOWSPEED_9V', 'RAWMODE','dontreply', NXT(n));
        statusSENSOR_4 = NXT_SetInputMode(SENSOR_4, 'LOWSPEED_9V', 'RAWMODE','dontreply', NXT(n));
    end
    clc 
%% INITIAL VARIABLES 
    Tstart = tic;
    Ttotal = 0;
   
numWT = 0;
for ii=1:4
    numWT = numWT + 1;
%	WT.name(1) =  nameFileExperiment;         %name of the experiment (string)
    WT(numWT).directMov =  directMov ;            %1 for clockwise, = 2 for counter clockwise
    WT(numWT).vertHoriz =  vertHoriz ;            %1 for vertical shaft, = 2 for horizontal shaft
    WT(numWT).radRot =     radRot;                %rotor radius (m)
    WT(numWT).areaRot =    areaRot;               %rotor area (m2). Classical is = pi*radRot^2
    WT(numWT).igSensorNumber =  igSensorNumber;   %number of sensor for current
    WT(numWT).vgSensorNumber =  vgSensorNumber;   %number of sensor for voltage
    WT(numWT).numGen =  numGen;                   %number of generator
    WT(numWT).Ra =  [];                           %internal resistance generator (ohm)
    WT(numWT).gam = [];                           %friction generator and drive-train (kg m2 / sec)
    WT(numWT).time = [];                     %seconds
    WT(numWT).VWT = [];                      %wind velocity, calibrated (m/s)
    WT(numWT).wrWT = [];                     %rotor velocity, calibrated (rad/sec)
    WT(numWT).igWT = [];                     %generator current, calibrated (A)
    WT(numWT).vgWT = [];                     %generator voltage, calibrated (volts)
    WT(numWT).Pwind = [];                    %wind power (watt)
    WT(numWT).Pmec = [];                     %shaft mechanical power (watt)
    WT(numWT).Pelec = [];                    %generator output electrical power (watt)
    WT(numWT).Cp = [];                       %power aerodynamic coefficient(per unit)
    WT(numWT).lambda = [];                   %tip speed ratio
    WT(numWT).effGenDT = [];                 %efficiency of generator and drive-train (per unit)
    WT(numWT).Ea = [];                       %internal voltage generator (volt)
    WT(numWT).KT = [];                       %torque constant generator (N m / amp)
    WT(numWT).vgRefWT = [];                  %vg reference for voltage control case (volt)
    WT(numWT).arduino = [];                  %arduino input (0 to 255)
    WT(numWT).uContr = [];                   %controller output
    WT(numWT).delta = [];                    %delta for gain scheduling controller 

end   
%% DATA ACQUISITION
% Time inicialization
    t(nTurb+nAdd) = 0; % Current time  % GENERALIZE
    tAnt(nTurb+nAdd) = 0; % Previous time  % GENERALIZE
    tic; % The time starts here
% Inicialization PIDs for current control
    uContrWT1_p = 0;
    WR_refWT1 = 0;
    ErrorWT1_ig_p = 0;
    errorWT1_WR1 = 0;
    Vrefn1 = 0 ;
    v1n1 = 0 ;
    u2n1= 0 ;
    xn1= 0 ;
    voltage = 0;
    current = 0;
    rotorVelocity1 = 0;
    firstTime = 2; % For plant ident
    W_change = 0;
    WR_refWT_matrix=[];
%%
    WXdata = [0,100]; % time for Vref
    WYdata = [8,8]; % Vref
    WindSpeed_p = 0;
    Ig_p = 0;
    P_P = 0;
    wr_P=0;
    V_P = 0;
    P_no1_PP = 0;
    P_no1_P = 0;
    P_ESPP = 0;
    P_ESP = 0;
    Ig_pp = Ig_p;
    P_PP = P_P;
    V_PP = V_P;
    SES_P = 0;
    PES_P = 0;
    P_no12_P=0;
    P_no1_P=0;
    Yy_P=0;
    uu_P=0;
    lambda_P = 2;
    lambda1 = 2;
    igSensor = 0;
    vgSensor = 0;
    wrSensor = 0;
    vvCu_P = 0;
    vvVo_P = 0;
    Cp_Low = 0;
    Cp_P = 0;
    vvOm_P = 0;

   deltaT = 0.15; % Sec
%% Loop. MAIN PROGRAM
    Delay = 0;
    nnn = 1;
    a1 = 0.3196;
    b1=3.5;
    c1=0.5;
    PAO = 0 ;
    Cp =0;
    CpDATAShift = zeros(1,5);

while Ttotal<100 % Doesn't work so in order to stop the program press any key   
     tstart = tic;
     WindSpeed =interp1(WXdata,WYdata,Ttotal);
     Wuchar1 = WindspeedArduino(WindSpeed);
     if WindSpeed ~= WindSpeed_p
        fwrite(sf,Wuchar1); % Send voltage to arduino
     end
     WindSpeed_p = WindSpeed;  
%%
    for n=1:nTurb    
        % dataSENSOR_1 (Rotor velocity measured by the sensor in RPM) (It is converted into rad/sec)(2 bytes)
        dataSENSOR_1 = COM_ReadI2C(SENSOR_1, 4, uint8(48), uint8(74), NXT(n));    
        if isempty(dataSENSOR_1)
            rotorVelocity1(n) = 0;
        else
            rotorVelocity1(n) = typecast(uint8([dataSENSOR_1(1) dataSENSOR_1(2)]), 'int16');
        end 
        Arotor(n) = (abs(double(rotorVelocity1(n))) * pi/30); % To be in rad/sec      
        % dataSENSOR_3 (Current measured by the sensor in miliAmperes)(It is converted into Amps)(4 bytes)
        dataSENSOR_3 = COM_ReadI2C(SENSOR_3, 4, uint8(40), uint8(67), NXT(n));
        if isempty(dataSENSOR_3)
            current(n) = 0;
        else
            current(n) = typecast(uint8([dataSENSOR_3(1) dataSENSOR_3(2)]), 'int16');
            current(n) = abs(current(n)); % To be in Amps
        end           
        % dataSENSOR_4 (Voltage in volts) (2 bytes)
        dataSENSOR_4 = COM_ReadI2C(SENSOR_4, 4, uint8(38), uint8(67), NXT(n));
        if isempty(dataSENSOR_4)
            voltage(n) = 0;
        else
            voltage(n) = typecast(uint8([dataSENSOR_4(1) dataSENSOR_4(2)]), 'int16');
            voltage(n) = abs(voltage(n))/1000; % from mVolts to Volts
        end  
        
    while  nnn <= 20
            vvPrevOm = Arotor(n); % rad/sec
            vvPrevCu = double(current(n)/1000); % A
            vvPrevVo = voltage(n); % Volt
            nnn = nnn+1
    end
        
%% Sensor Protection
    % Omega sensor (1)
    [vvOm,vvPrevOm] = protectionSensorValue(Arotor(n),vvPrevOm,deltaT,1);
    % Current sensor (3)
    [vvCu,vvPrevCu] = protectionSensorValue(double(current(n)/1000),vvPrevCu,deltaT,3);
    % Voltage sensor (4)
    [vvVo,vvPrevVo] = protectionSensorValue(double(voltage(n)),vvPrevVo,deltaT,4);         
%% Update - time
        tAnt(n) = t(n); %Previous time
        t(n) = toc; % currentTime % CHANGE AFTER!!!! 
        

     end
% igSensor = vvCu;
% vgSensor = vvVo;
% wrSensor = vvOm;
% %0.2s
% igSensor = 0.7778*igSensor+0.1111*(vvCu+vvCu_P);
% vgSensor = 0.7778*vgSensor+0.1111*(vvVo+vvVo_P);
% wrSensor = 0.7778*wrSensor+0.1111*(vvOm+vvOm_P);
%0.5s
% igSensor = 0.9048*igSensor+0.04762*(vvCu+vvCu_P);
% vgSensor = 0.9048*vgSensor+0.04762*(vvVo+vvVo_P);
% wrSensor = 0.9048*wrSensor+0.04762*(vvOm+vvOm_P);
% 0.25s
% igSensor = 0.8182*igSensor+0.09091*(vvCu+vvCu_P);
% vgSensor = 0.8182*vgSensor+0.09091*(vvVo+vvVo_P);
% wrSensor = 0.8182*wrSensor+0.09091*(vvOm+vvOm_P);
% 0.4s
% igSensor = 0.8824*igSensor+0.05882*(vvCu+vvCu_P);
% vgSensor = 0.8824*vgSensor+0.05882*(vvVo+vvVo_P);
% wrSensor = 0.8824*wrSensor+0.05882*(vvOm+vvOm_P);
%%1s Working
% igSensor = 0.9512*igSensor+0.02439*(vvCu+vvCu_P);
% vgSensor = 0.9512*vgSensor+0.02439*(vvVo+vvVo_P);
% wrSensor = 0.9512*wrSensor+0.02439*(vvOm+vvOm_P);
%%3s
% igSensor = 0.9835*igSensor+0.008264*(vvCu+vvCu_P);
% vgSensor = 0.9835*vgSensor+0.008264*(vvVo+vvVo_P);
% wrSensor = 0.9835*wrSensor+0.008264*(vvOm+vvOm_P);

%%5s
igSensor = 0.99*igSensor+0.004975*(vvCu+vvCu_P);
vgSensor = 0.99*vgSensor+0.004975*(vvVo+vvVo_P);
wrSensor = 0.99*wrSensor+0.004975*(vvOm+vvOm_P);

%%10s
% igSensor = 0.995*igSensor+0.002494*(vvCu+vvCu_P);
% vgSensor = 0.995*vgSensor+0.002494*(vvVo+vvVo_P);
% wrSensor = 0.995*wrSensor+0.002494*(vvOm+vvOm_P);

%%3s
% igSensor = 0.9512*igSensor+0.02439*(vvCu+vvCu_P);
% vgSensor = 0.9512*vgSensor+0.02439*(vvVo+vvVo_P);
% wrSensor = 0.9512*wrSensor+0.02439*(vvOm+vvOm_P);
        vvCu_P = vvCu;
        vvVo_P = vvVo;
        vvOm_P = vvOm;
[VWT,wrWT,igWT,vgWT,Pwind,Pmec,Pelec,Cp,lambda,effGenDT,Ea,KT,Ra,gam] = dataOnLineProcessingMGS1(WindSpeed,wrSensor,igSensor,vgSensor,igSensorNumber,vgSensorNumber,numGen,directMov,vertHoriz,radRot,areaRot);
%%

% switch WindSpeed
%     case 6.2
%         Cp =  0.188*exp(-((lambda-1.912)/1.218)^2)
%     case 7.1
%          Cp =  0.188*exp(-((lambda-1.97)/1.225)^2)
%     case  8
%          Cp =  0.188*exp(-((lambda-2.084)/1.028)^2)
%     case 8.35
%          Cp =  0.1888*exp(-((lambda-2.084)/0.8453)^2)
%     case 8.85
%          Cp =  0.1884*exp(-((lambda-2.163)/0.6704)^2)
% end
%          Cp_Low = 0.9835*Cp_Low+0.008264*(Cp+Cp_P);
%          Cp_P = Cp;
%          Cp = Cp_Low
Cp
        PES =Pmec;%mean(CpDATAShift);
        K2 = 2;
        k1 = 8;
        delta_Ig = igWT -Ig_p;
        Slope = delta_Ig/0.3;

        if Slope >50
            Delta_MPPT_V_ref = Vg + K2*Slope;
        else
            delta_P = PES - P_P;
            delta_V = vgWT - V_P;
            delta_omega = wrWT-wr_P;
            Direction = delta_P*delta_omega;
            if Direction <= 0
                Delta_MPPT_V_ref = -k1*0.2;
            else
                Delta_MPPT_V_ref = +k1*0.2;
            end
        end  
        MPPT_V_ref = Delta_MPPT_V_ref + vgWT;
        Ig_pp = Ig_p;
        P_PP = P_P;
        V_PP = V_P;
        Ig_p = igWT;
        P_P = PES;
        V_P = vgWT;
        Ref = MPPT_V_ref;
%% S-2
        v1n = WindSpeed;
        Vref = 0.9512*Vrefn1+0.02122*(v1n+v1n1);
        Vrefn1 = Vref;
        v1n1 = v1n;
%% S-3
        xn=WindSpeed-8;
        u2 = 0.9512*u2n1+0.02122*(xn+xn1);
        u2n1 = u2;   
        xn1 = xn;
%% S-4     PID for Vg control
        Vg_sensor = max(0.00001,abs(voltage(1)));
        if (Ref <= 2)   
            delta = 1;
        elseif (Ref <= 7) &&  (Ref > 2)
             delta = 0.75; %1 Work for Wind Speed 6 & 7, 0.75 WOrk For 8m/s
        else
            delta = 1/3;
        end
        a_WT = -0.4556*delta;
        b_WT = 0.4333*delta;
        ErrorWT1_ig=Ref-Vg_sensor;
        uContrWT = uContrWT1_p +a_WT* ErrorWT1_ig + b_WT* ErrorWT1_ig_p; % ts = 2 sec
        uContrWT = max(0,uContrWT);
        uContrWT1_p = uContrWT;
        ErrorWT1_ig_p = ErrorWT1_ig;
%% To Atduino;
        uContrWT1 = double(uContrWT) ; 
%% Write to Arduino (u) 
        [uchar1,u1]= Arduino_input(uContrWT1);
        fwrite(s,uchar1); % Send voltage to arduino
        u1

%%  Loop: wait and write
    WT(n).Ra(end+1) =  Ra;                      %internal resistance generator (ohm)
    WT(n).gam(end+1) = gam;                      %friction generator and drive-train (kg m2 / sec)
    WT(n).VWT(end+1) = WindSpeed;                      %wind velocity, calibrated (m/s)
    WT(n).wrWT(end+1) = wrWT ;                     %rotor velocity, calibrated (rad/sec)
    WT(n).igWT(end+1) = igWT;                     %generator current, calibrated (A)
    WT(n).vgWT(end+1) = vgWT;                     %generator voltage, calibrated (volts)
    WT(n).Pwind(end+1) = Pwind;                    %wind power (watt)
    WT(n).Pmec(end+1) = Pmec;                     %shaft mechanical power (watt)
    WT(n).Pelec(end+1) = Pelec;                    %generator output electrical power (watt)
    WT(n).Cp(end+1) = Cp;                       %power aerodynamic coefficient(per unit)
    WT(n).lambda(end+1) = lambda;                   %tip speed ratio
    WT(n).effGenDT(end+1) = effGenDT;                 %efficiency of generator and drive-train (per unit)
    WT(n).Ea(end+1) = Ea;                       %internal voltage generator (volt)
    WT(n).KT(end+1) = KT;                       %torque constant generator (N m / amp)
    WT(n).vgRefWT(end+1) = Ref;                  %vg reference for voltage control case (volt)
    WT(n).arduino(end+1) = u1;                  %arduino input (0 to 255)
    WT(n).uContr(end+1) = uContrWT1;                   %controller output
    WT(n).delta(end+1) = delta;                    %delta for gain scheduling controller 

    tfin = toc(tstart);  % Time that takes to run the program in the loop
    twait = 0.05-tfin;     % Time to wait for reach the sample time. In this case is 0.12
    pause(twait);       % We wait for getting that time
    Ttotal = toc(Tstart); % Total time
    
    WT(n).time(end+1) = Ttotal;                     %seconds
end
%%
cc = ceil(clock);
nameFileExperiment2 = [num2str(cc(1)) num2str(cc(2)) num2str(cc(3)) '_' num2str(cc(4)) num2str(cc(5)) '_' num2str(cc(6)) '_WF_experiment.mat']; 
save(nameFileExperiment2, 'WT');
%% Set Wind Tunnel to 15HZ

Wu1 = round(2000); % Round the value for getting an integer in order to send it to Arduino
Wuc1 = num2str(Wu1); % Convertion of integer into char
Wuchar1 = ['(',Wuc1,')']; % Preparation for sending the data to Arduino
fwrite(sf,Wuchar1); % Send voltage to arduino
W_change = 1;
            
            
fclose(s); % Close the serial connection between arduino and matlab
fclose(sf); % Close the serial connection between arduino and matlab
close all;
COM_CloseNXT all; % Close connection with NXT




    