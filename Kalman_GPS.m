%Path estimation of a vehicle by gps using the Kalman Filter:

close all
clear all

duration = 20; %Duration of the time that the GPS try to estimate the
%position of the vehicle.
dt = .1;%How often the GPS detects the signal from the correct position of
%the vehicle


X_loc = []; %Initialization of the actual position of the vehicle
X_loc_GPS = []; %Location of the Vehicle according to the GPS
Y_matrix = zeros(1,duration/dt);%Initialization of the matrix Y

%Now the Kalman Filter Ecuations are defined

A = [1 dt; 0 1]; %State stransition matrix: Prediction of the state of the
%vehicle
B = [dt^2/2; dt];%Input Control matrix: Expected effect of the input
%aceleration in the state of the vehicle
C = [1 0];% Measurement matrix.

a = 0.7; %Aceleration of the vehicle (let's suppose that is constant)
X = [0; 0]; %Initial state of the vehicle; [posistion; velocity]
X_estimate = X; %Initailization of the estimation
Vehicle_noise_magnitude = 0.05; %Noise in the speed of the vehicle
GPS_noise_magnitude = 10; %Noise in the detection of the GPS
Evehicle = Vehicle_noise_magnitude^2 * [dt^4/4 dt^3/2; dt^3/2 dt^2]; %Covariance in the movement of the vehicle
EGPS = GPS_noise_magnitude^2; %Covariance of what the GPS man detects
P = Evehicle; %Estimation of the initial vehicle position variance
X_loc_estimate = []; %Vehicle position matrix estimated by the Kalman


for t = 0: dt : duration
    %The noise change randomly
    Vehicle_noise = Vehicle_noise_magnitude * [randn; randn];
    GPS_noise = GPS_noise_magnitude * randn;
    %The Kalman Filter's ecuations are applied
    X = A*X + B*a + Vehicle_noise;
    Y = C*X + GPS_noise;
    %The values obtained are setted in the corresponding place of the matrix
    X_loc = [X_loc; X(1)];
    X_loc_GPS = [X_loc_GPS; Y];
    
    plot(0:dt:t, X_loc, '-r.');
    plot(0:dt:t, X_loc_GPS, '-k.');
    axis([0 20 -30 100]);
    hold on
    
end

plot(0:dt:t, smooth(X_loc_GPS), '-g.');

X= [0; 0]; % re-initized X


for t = 1:length(X_loc)
   %PREDICTION PART
    X_estimate = A * X_estimate + B * a;
    P = A * P * A' + Evehicle; 
    %UPDATE PART 
    K = (P*C')/(C*P*C'+EGPS);
    X_estimate = X_estimate + K * (X_loc_GPS(t) - C * X_estimate);
    P =(eye(2)-K*C)*P;
     
    X_loc_estimate = [X_loc_estimate; X_estimate(1)];
end

figure(2);
tt = 0 : dt : duration;
plot(tt,X_loc,'-r.',tt,X_loc_GPS,'-k.', tt,X_loc_estimate,'-g.');
axis([0 20 -30 100])



