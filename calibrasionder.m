clear all;clc; close all;

robot = 'Pioneer3AT';
laser = 'LMS100';
addpath 'C:\Users\Usuarie\Desktop\Apolo\Matlab'

Xini = [8; 1.1; pi/2];

apoloPlaceMRobot(robot, [Xini(1) Xini(2) 0], Xini(3));
apoloResetOdometry(robot, Xini');

velocidadL = 0.2; % velocidad lineal [m/s]
timestep = 0.5; % periodo del filtro [s]
subpasos = 5; % Apolo mueve en pasos de 0.1 s para no "saltarse" obstaculos
radio = 0.8; % circulo centrado en (8, 4.2): no toca cajas ni estanterias
numerodepasos = ceil(2*pi*radio/(velocidadL*timestep));
velocidadA = 2*pi/(numerodepasos*timestep); % velocidad angular [rad/s]
nPasos = 2*numerodepasos; % dos vueltas


c  = cos(Xini(3));
s = sin(Xini(3));
xs = Xini(1) + 0.1*c;                 % posicion del laser
ys = Xini(2) + 0.1*s;

bx = 2.640;
by = 11.290;
dx = bx - xs;
dy = by - ys;
q  = dx^2 + dy^2;
r  = sqrt(q);
ang =  atan2(dy,dx) - Xini(3);


dist = zeros(1);
angl = zeros(1);

for i=1:100
    lm = apoloGetLaserLandMarks(laser);
    d = lm.distance(1);
    a = lm.angle(1);

    dist(i) = d;
    angl(i) = a;
end

mean_dist = mean(dist - r)
var_dist = var(dist - r)
mean_angl = mean (angl - ang)
var_angl = var(angl - ang)