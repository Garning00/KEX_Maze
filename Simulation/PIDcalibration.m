clear all
s = tf('s');
g = 9.81;
K_P = 0.5;
K_D = 0.5;
K_I = 1;


G_plan = (3/5)*g/s^2;
F = K_P + K_I/s + K_D*s;
%G_motor = ;

Gc = feedback( F * G_plan, 1 );
step(Gc);

