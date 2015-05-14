% Code for testing the atmospheric model

close all
clear all

% Global Variables
global ALTITUDE
altitude = ALTITUDE

ALTITUDE = 0.0;
velocity = 20;
k = 1;

for i = 0:1000:100000
    ALTITUDE = i;
    [windspeed, P(k), rho(k), T(k), mu, nu, mach, Re, q, a] = atmosphere(velocity);
    k = k+1;
end

y = 0:100;
figure(1)
plot(rho,y)
title('Altitude vs. rho')
figure(2)
plot(P,y)
title('Altitude vs. Pressure')
figure(3)
plot(T,y)
title('Altitude vs. Temperature')