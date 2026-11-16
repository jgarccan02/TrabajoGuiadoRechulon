clear all;clc; close all;

robot = 'Pioneer3AT';
laser = 'LMS100';
addpath 'C:\Users\Usuarie\Desktop\Apolo\Matlab'

Xini = [8; 1.1; pi/2];

apoloPlaceMRobot(robot, [Xini(1) Xini(2) 0], Xini(3));
apoloResetOdometry(robot, Xini');

velocidadL = 0.2 ; % velocidad lineal [m/s]
timestep = 0.5; % periodo del filtro [s]
subpasos = 5; % Apolo mueve en pasos de 0.1 s para no "saltarse" obstaculos
radio = 0.8; % circulo centrado en (8, 4.2): no toca cajas ni estanterias
numerodepasos = ceil(2*pi*radio/(velocidadL*timestep));
velocidadA = 2*pi/(numerodepasos*timestep); % velocidad angular [rad/s]
nPasos = 2*numerodepasos; % dos vueltas

odometriaL = zeros(1);
realL = zeros(1);

odometriaA = zeros(1);
realA = zeros(1);

for i=1:100

    odo_prev = apoloGetOdometry(robot);
    p_prev = apoloGetLocationMRobot(robot);

    apoloMoveMRobot(robot, [velocidadL velocidadA], timestep/subpasos);

    odo = apoloGetOdometry(robot);
    p = apoloGetLocationMRobot(robot);

    velocidadL_odo  = hypot(odo(1)-odo_prev(1), odo(2)-odo_prev(2))/(timestep/subpasos);
    velocidadL_p  = hypot(p(1)-p_prev(1), p(2)-p_prev(2))/(timestep/subpasos);

    odometriaL(i) = velocidadL_odo;
    realL(i) = velocidadL_p;

    velocidadA_odo = atan2(sin(odo(3)-odo_prev(3)), cos(odo(3)-odo_prev(3)))/(timestep/subpasos);
    velocidadA_p = atan2(sin(p(4)-p_prev(4)), cos(p(4)-p_prev(4)))/(timestep/subpasos);

    odometriaA(i) = velocidadA_odo;
    realA(i) = velocidadA_p;

end

mediaL = mean(odometriaL - realL)
varL = var(odometriaL - realL)
mediaA = mean(odometriaA - realA)
varA = var(odometriaA - realA)