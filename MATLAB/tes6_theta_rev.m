% notes yg digunakan apabila diketahui fungsi transfer sebagai berikut
% - alpha-theta, (T)
% - pwmSteer-alpha, (A)
% - adcSignal-speed (S)
% 
%     A1=1;   A2=0;   A3=0;
%     T1=0;   T2=0;   T3=0;
%     S1=0;   S2=0;   S3=0;
%     alpha_i = A1*pwmA + A2*prev_alpha_i + A3*alpha(i-2);
%     theta(i) = T1*prev_alpha_i + T2*theta(i-1) + T3*theta(i-2);
%     prev_Vs_i = S1*pwmS + S2*Vs(i-1) + S3*Vs_i;
%%REVISI antara tes3 dan tes4 respon  belok brbeda
% Description :
% theta : angle of vehicle - sensor:CMPS12
% angle : steering angle of vehic;e - sensor:Hall sensor
% Vs_i : speed of vehicle - sensor: Hall sensor
% (x,y) : posiition on the road - sensor:GPS (lat,long)

clear;
clc;

load('maps13_kaka.mat'); %maps11 maps12
% PID Constant
Kd1=6;%100.2; %40.2
Ki1=0.0; %0.0008;; %konstanta alpha pengali error theta
Kp1=0.17;%0.5; %konstanta alpha pengali roadError
Ktheta = (pi/1.2);
Kp1Turn = 1.62;%1.62;%4*Kp1;
Kd1Turn = 0;

%index/parameter
samplingTime=0.05;
index = 12;%9; %4
maxIndex = 13;
init_theta = 15;%15;
arduino = 0;
fps = 3;  


%initialize data
d = 1.8;    % Jarak Roda Depan dan Belakang
Vmax = 4.0; % maxSpeed
x_tot = 2.5;% lebar jalan 2,5 m
distTolerance = 15; %distance for deceleration
turnAngle=40;

sumThetaError = 0;
alphaMax=27.5;
t_alpha_i = 0;
alpha_i = 0;  % Sudut Steering Tricycle
prev_alpha_i = alpha_i;
Wa_max = 6;%6 aslinya
belok=0;

acc = 0.3;

err_pos1 =0;
err_v2 = 0;
sp_pos=0;
sp_v2=0;

prev_Vs_i = 0;     % kecepatan linear kendaraan
Vs_i = 0;       % kecepatan linear kendaraan
% configure serial port
if (arduino)
    ser_port = serialport("COM6", 115200);
    flush(ser_port);
    configureTerminator(ser_port,59,"CR/LF"); %59 = ';'
    pause(10);
end




speedLimit = 4*samplingTime;%0.2;%Vmax;
i=1;
x(i) = maps(index-1).x;%1.25;   % Posisi X, peretengahan dengan lebar jalan 2,5 Meter
x(i+1) = x(i);
x(i+2) = x(i);
y(i) = maps(index-1).y;%0;    % Posisi Y
y(i+1) = y(i);
y(i+2) = y(i);
x_ahead_i = x(i);
y_ahead_i = y(i);
prevX = maps(index-1).x;
prevY = maps(index-1).y;
targetX = maps(index).x;
targetY = maps(index).y;
nextX = maps(index+1).x;
nextY = maps(index+1).y;
roadError = 1.3 - x_ahead_i; 


if (targetX>prevX)
    targetHeading_i = atan((targetY - prevY)/(targetX - prevX));
elseif (targetX<prevX)
    targetHeading_i = atan((targetY - prevY)/(targetX - prevX)) - pi;
end
targetHeading_i = modx(targetHeading_i,1);

targetTheta_i = targetHeading_i;
targetAlpha_i = 0;

for i=1:3
    time(i) = 0;
    Kec_i = 0;
    theta(i) = targetHeading_i + degtorad(init_theta);
    targetTheta_i=targetHeading_i;
end
north =1;

i=2;
while (i<2500)
    t_head(i) = radtodeg(targetHeading_i);
    speedTargetMax=0;
    alphadeg(i) = radtodeg(alpha_i);
    errorDist_i = sqrt( (targetY - y(i))^2 + (targetX - x(i))^2 );
    
    if (targetX>=prevX)
        targetHeading_i = atan((targetY - prevY)/(targetX - prevX));
    else
        targetHeading_i = atan((targetY - prevY)/(targetX - prevX)) - pi;
    end
    targetHeading_i = modx(targetHeading_i,1);
%     if (targetHeading_i>=pi) 
%         targetHeading_i = -2*pi+targetHeading_i;
%     elseif (targetHeading_i<=-1*pi)
%         targetHeading_i = -2*pi-targetHeading_i;
%     end
    errorHeading(i) = calcError(targetHeading_i,theta(i),1);
%     radtodeg(abs(errorHeading(i)))
    if (abs(errorHeading(i))>=degtorad(turnAngle))
        belok=1;
        sumThetaError=0;
    elseif (abs(errorHeading(i))<degtorad(5)) %turn is over when theta error is less than x degrees
        belok=0;
        sumThetaError = 0;
    end
    
    
    if (abs(targetHeading_i) <= degtorad(30))
        roadError = (targetY - y_ahead_i );
    elseif ( ((targetHeading_i) <= degtorad(120)) && ((targetHeading_i) >= degtorad(60)) ) 
        roadError = (-targetX) + x_ahead_i; % 40  + 76
    elseif ( (abs(targetHeading_i) <= degtorad(210)) && (abs(targetHeading_i) >= degtorad(150)) ) 
        roadError = (-targetY + y_ahead_i); % 40  + 76
    elseif ( ((targetHeading_i) >= degtorad(-120)) && ((targetHeading_i) <= degtorad(-60)) ) 
        roadError = (targetX - x_ahead_i); % 40  + 76
    end
    
    if (belok==0)
        if (abs(errorHeading(i))<degtorad(turnAngle))
            if (abs(roadError)<=0.2)
                speedTargetMax = 4*samplingTime ;
            else
                speedTargetMax = ( 0.5 + 0.5/abs(roadError))*samplingTime; 
            end
        else 
            speedTargetMax = 0.5*samplingTime;
        end
        maxFact = degtorad(90);
        roadFactor = (roadError*Ktheta);
%         if ((roadFactor)>maxFact)
%             roadFactor = maxFact;
%         elseif ((roadFactor)<-1*maxFact)
%             roadFactor = -1* maxFact;
%         end
        targetTheta_i = targetHeading_i + roadFactor;
    elseif (belok)
        targetTheta_i = targetHeading_i;
        sumThetaError=0;
        speedTargetMax = 1.20*samplingTime; %speed when vehicle take a turn
        
    end
%     targetTheta_i = threshold(targetTheta_i,180,1);
    % -190
%     while(abs(targetTheta_i)> editted
%     if (targetTheta_i>degtorad(180))
%         targetTheta_i = targetTheta_i-degtorad(180);
%     elseif (targetTheta_i<=-1*degtorad(180))
%         targetTheta_i = targetTheta_i+degtorad(180);
%     end
%     %speedTarget Limit
    if (speedTargetMax>4*samplingTime)
        speedTargetMax = 4*samplingTime;
    elseif (speedTargetMax<-4*samplingTime)
        speedTargetMax = -4*samplingTime;
    end
%     if (targetTheta_i<0)
%         targetTheta_i = degtorad(180) - targetTheta 
%           -179 - 90 = -269, 181 - 90 = 89, -90 - 
%         -150 - 90 = -240 ~ 360+(-240) = 120  ::::::170 - (-90) 260 --- (180 - 260) = -80
% ed22

    errorTheta(i+1) = targetTheta_i - theta(i);
    if ( (belok) && radtodeg(errorTheta(i+1))>180)
        errorTheta(i+1) = errorTheta(i+1) - degtorad(360);
    elseif ( (belok) && radtodeg(errorTheta(i+1))<-180)
        errorTheta(i+1) = errorTheta(i+1) + degtorad(360);
    end
%     tes55(1) = radtodeg(errorTheta(i+1));
%     tes55(2) = radtodeg(targetTheta_i);
%     tes55(3) = roadError;
%     errorHeading(i) = calcError(targetHeading_i,theta(i),1);
%     tes55(i,1) = radtodeg(errorHeading(i));
%     tes55(i,2) = radtodeg(targetHeading_i);
%     tes55(i,3) = radtodeg(theta(i));
%     tes55
%     if (errorTheta(i+1)>degtorad(180))
%         errorTheta(i+1)=errorTheta(i+1) - degtorad(360);
%     elseif (errorTheta(i+1)<=-1*degtorad(180))
%         errorTheta(i+1)=errorTheta(i+1) + degtorad(360);
%     end
%     errorTheta(i+1) = calcError(targetTheta_i, theta(i),1);
%     if ( abs(targetTheta_i - theta(i)) < abs(degtorad(360) + targetTheta_i - theta(i)) )
%         errorTheta(i+1) = targetTheta_i - theta(i);
%     else
%         errorTheta(i+1) = degtorad(360) + targetTheta_i - theta(i);
%     end


%     if (abs(errorTheta(i+1)+360) < abs(errorTheta(i+1))
%     while (abs(errorTheta(i+1))>=degtorad(180))
%         if (errorTheta(i+1)>=degtorad(180))
%             errorTheta(i+1) = degtorad(180) - errorTheta(i+1);
%         elseif (errorTheta(i+1)<=-1*degtorad(180))
%             errorTheta(i+1) = degtorad(180) + errorTheta(i+1);
%         end
%     end
    if (abs(roadError)<0.05)
        sumThetaError = 0;
    else 
        sumThetaError = sumThetaError + errorTheta(i+1);
        if (sumThetaError> alphaMax/Ki1)
            sumThetaError= alphaMax/Ki1;
        elseif (sumThetaError< -1*alphaMax/Ki1)
            sumThetaError= -1*alphaMax/Ki1;
        end
    end
    if (belok==0)
        targetAlphaMax = Kp1*errorTheta(i+1) + Ki1*sumThetaError + Kd1*(errorTheta(i+1) - errorTheta(i));
    else
        targetAlphaMax = Kp1Turn*errorTheta(i+1)  + Kd1Turn*(errorTheta(i+1) - errorTheta(i));%+ Ki1*sumThetaError 
        sumThetaError = 0;

    end
    if (targetAlphaMax>degtorad(alphaMax))
        targetAlphaMax=degtorad(alphaMax);
    elseif (targetAlphaMax<-1*degtorad(alphaMax))
        targetAlphaMax=-1*degtorad(alphaMax);
    end
    
    % alpha speed 6 degree /s
    errorAlpha_i = targetAlphaMax - t_alpha_i;
    % generate alpha target
    if ( (radtodeg(errorAlpha_i))>Wa_max*samplingTime )
        t_alpha_i = t_alpha_i + degtorad(Wa_max*samplingTime);
    elseif ( (radtodeg(errorAlpha_i))<-Wa_max*samplingTime )
        t_alpha_i = t_alpha_i - degtorad(Wa_max*samplingTime);
    else
        t_alpha_i = targetAlphaMax;
    end
    if (arduino==0)
        alpha_i=t_alpha_i;
    end
    
    if (errorDist_i < distTolerance)
        if (belok==0)
            speedTargetMax = speedTargetMax * (errorDist_i/(distTolerance));
        end
        if (errorDist_i<=4.5 && index<maxIndex)
            index = index+1;
            prevX = maps(index-1).x;
            prevY = maps(index-1).y;
            targetX = maps(index).x;
            targetY = maps(index).y;
            nextX = maps(index+1).x;
            nextY = maps(index+1).y;
            
        elseif (speedTargetMax<0.2*samplingTime)
            speedTargetMax=0;
        end     
    end
    
    if (arduino==0) %simulation only, generate Vs_i value
        if ( speedTargetMax<prev_Vs_i)
            Vs_i = prev_Vs_i-0.1*samplingTime;
        elseif ( speedTargetMax-prev_Vs_i<=0.1*samplingTime)
            Vs_i = speedTargetMax;
        elseif ( speedTargetMax>prev_Vs_i)
            Vs_i = prev_Vs_i+0.1*samplingTime;
        end
    end
    % generate acceleration - 0.3 m/s2
    if ( speedTargetMax<prev_Vs_i-acc*samplingTime)
        speedAccTarget_i = prev_Vs_i-acc*samplingTime;
    elseif (abs( speedTargetMax-prev_Vs_i)<=acc*samplingTime)
        speedAccTarget_i = speedTargetMax;
    elseif ( speedTargetMax>prev_Vs_i + acc*samplingTime)
        speedAccTarget_i = prev_Vs_i+acc*samplingTime;
    end
%     targetY = -100;
%     prevY = 0;
%     targetX = -25;
%     prevX = 0;
%     if (targetX>=prevX)
%         targetHeading_i = atan((targetY - prevY)/(targetX - prevX));
%     else
%         targetHeading_i = atan((targetY - prevY)/(targetX - prevX)) - pi;
%     end
%     targetHeading_i = modx(targetHeading_i,1);
    
%     alpha_i=degtorad(-20);
    
%     theta(i)=degtorad(-180);
    % update Tricycle Parameter
    dx = Vs_i*cos(alpha_i)*cos(theta(i));
    dy = Vs_i*cos(alpha_i)*sin(theta(i));
    dTheta = (Vs_i*sin(alpha_i)/d);
    
    x(i+1) = x(i) + dx;
    y(i+1) = y(i) + dy;
    x_ahead_i = x(i) + ((4+1.5)*samplingTime*cos(alpha_i)*cos(theta(i)))/samplingTime;
    y_ahead_i = y(i) + ((4+1.5)*samplingTime*cos(alpha_i)*sin(theta(i)))/samplingTime;
    theta(i+1) = theta(i) + dTheta;
    if (theta(i+1)>=pi)
        theta(i+1) = degtorad(181);
    elseif (theta(i+1)<=-pi)
        theta(i+1) = -1*degtorad(181);
%     elseif (theta(i+1)<=degtorad(1))
%         theta(i+1) = -1*degtorad(1);
%     elseif (theta(i+1)>=degtorad(-1))
%         theta(i+1) = degtorad(1);
    end
    Kec_i = Vs_i/samplingTime;
    i=i+1;
%     if (abs(theta(i))<=degtorad(1) || (abs(theta(i))>degtorad(179) && abs(theta(i))<degtorad(180)))
%         theta(i) = degtorad(1);
%     end
    prev_alpha_i = alpha_i;
    prev_Vs_i = Vs_i;
    
    
    % draw maps, track and vehicle
    if (mod(i,fps) == 0)
        clf;
        plot(x,y,'LineWidth',2); % plot tricycle track simulation 
        hold on
        plot(x_ahead_i,y_ahead_i,'o'); % plot tricycle track simulation 
        hold on
        plot(x(i),y(i),'o');  % center of tricycle  
        draw_rectangle([x(i),y(i)],1.2,2.7,radtodeg(theta(i))-90,'red');    %draw rectangular using tricycle parameter
        
        % draw road line
        plot([targetX-0.3 targetX+0.3],[targetY targetY], "green", 'LineWidth',1);  %draw road line
        plot([0 0],[-30 102.6], "black", 'LineWidth',5);  %draw road line
        plot([2.6 2.6],[-30 100],"black", 'LineWidth',5); %draw road line
        plot([0 90],[82.6 82.6], "black", 'LineWidth',5);  %draw road line to right dir
        plot([2.6 90],[80 80],"black", 'LineWidth',5); %draw road line to right dir
        plot([40 40],[-100 150], "black", 'LineWidth',5);  %draw road line to right dir
        plot([42.6 42.6],[-100 150],"black", 'LineWidth',5); %draw road line to right dir

        % set max and min x-y axis in simulation
        axis([x(i)-5.75 x(i)+5.75 y(i)-2 y(i)+6]);   % [xmin xmax ymin ymax] limit
        % display value 
        text(x(i)-5.51,y(i)+5.5,sprintf('VsTarget = %.2f',speedAccTarget_i/samplingTime));%Kp1) );
        text(x(i)-5.51,y(i)+5.1,sprintf('Vs = %.2f m/s',Vs_i/samplingTime) );
        text(x(i)-5.51,y(i)+4.7,sprintf('errVs = %.2f',err_v2) );
        text(x(i)-5.51,y(i)+4.3,sprintf('alphaTarget = %.2f',radtodeg(targetAlphaMax)));
        text(x(i)-5.51,y(i)+3.9,sprintf('alpha = %.2f',radtodeg(alpha_i)) );
        text(x(i)-5.51,y(i)+3.0,sprintf('thetaTarget= %.2f',radtodeg(targetTheta_i)) );
        text(x(i)-5.51,y(i)+2.6,sprintf('theta = %.2f',radtodeg(theta(i))));
%         text(x(i)-5.51,y(i)+2.2,sprintf('KtBelok = %.2f',Ktheta));
        text(x(i)-5.51,y(i)+2.2,sprintf('Belok = %.2f',dy*100));

        text(x(i)-5.51,y(i)+1.8,sprintf('errTheta = %.2f',radtodeg(errorTheta(i))));
        text(x(i)-5.51,y(i)+1.4,sprintf('tHead = %.2f',radtodeg(targetHeading_i)));
        text(x(i)-5.51,y(i)+1.0,sprintf('ROAD = %.2f',roadError));
        text(x(i)-5.51,y(i)+0.6,sprintf('errHead = %.2f',radtodeg(errorHeading(i-1))));
        text(x(i)-5.51,y(i)+0.2,sprintf('%.2f %.2f',x_ahead_i,targetX));


        
        % draw green line - ideal/target track of vehicle
        if (abs(targetHeading_i)<pi/4 || abs(targetHeading_i)>pi*3/4) %saat belok kanan 45-135
            plot([prevX targetX],[targetY targetY], "green", 'LineWidth',1);  %draw Target Position
        else
            plot([targetX targetX],[targetY prevY], "green", 'LineWidth',1);  %draw Target Position
        end
        
        daspect([6 8 1]); % scale x : y : z
        title("simulation");
    end 
    
    if (arduino==1)%  WRITE and READ serial data from arduino
        t_pos = -1*round( radtodeg(t_alpha_i)*4*1425/360 ); 
        speed = round(speedAccTarget_i/samplingTime * 212/4); % maxspeed 200/TS
        %write and read data to arduino
        a= string(t_pos)+ "a" + string(speed) + "vp\n"
        writeline(ser_port,a); %command to arduino  
        pause(0.03); 
        if (ser_port.NumBytesAvailable > 0)
            sp_pos = str2double(readline(ser_port));
            sp_v2 = str2double(readline(ser_port));
            output = [i t_pos speed sp_pos sp_v2]; % print in/out data from/to arduino
        else
            go = false;
            disp ("connection error, no data received...")
            Vs_i = 0;
            alpha_i = 0;
            a= "0a0vp\n";
            writeline(ser_port,a); %command to arduino  
        end 
        Vs_i = sp_v2*samplingTime/(212/4);
        alpha_i = degtorad((-1*sp_pos)/(4*1425/360));
        err_pos1 = radtodeg(t_alpha_i) - (-1*sp_pos)/(2*1425/360);
        err_v2 = (speedAccTarget_i - Vs_i)/samplingTime;
    else
        pause(0.02);
    end
end

    


