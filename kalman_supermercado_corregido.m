close all;clc;clear all
%% EKF en el supermercado con Apolo
% Antes de ejecutar:
%   1. Abrir supermercado_apolo.xml en Apolo.
%   2. Tener en el path de MATLAB la carpeta con las funciones apolo*.m que vienen con Apolo.
%   3. ekf_update_corregido.m en esta misma carpeta.
%
% Apolo mueve el robot, genera la odometria (con su ruido) y mide las balizas.
% REVISION v5: trayectoria de odometria integrada desde Xini.
% Evita asumir que las componentes x,y crudas estan en ejes globales.
% No cambia la prediccion ni la correccion EKF.
% MATLAB predice con la odometria y corrige con las balizas que ve el laser.
% La pose real (apoloGetLocationMRobot) se usa SOLO para las graficas, nunca en el filtro.

robot = 'Pioneer3AT';
laser = 'LMS100';
rutaApolo = 'C:\Users\Usuarie\Desktop\Apolo\Matlab';
if exist(rutaApolo,'dir'), addpath(rutaApolo); end
assert(exist('apoloGetOdometry','file')~=0, ...
       'No encuentro las funciones de Apolo. Ajusta rutaApolo.');

%% Trayectoria circular (la del ejemplo), por la zona libre delante de las estanterias
velocidadL = 0.2; % velocidad lineal [m/s]
timestep = 0.5; % periodo del filtro [s]
subpasos = 5; % Apolo mueve en pasos de 0.1 s para no "saltarse" obstaculos
radio = 0.8;                    % radio nominal [m]
centroCirculo = [8; 4.2];         % centro deseado [m]
numerodepasos = ceil(2*pi*radio/(velocidadL*timestep));
velocidadA = 2*pi/(numerodepasos*timestep); % exactamente una vuelta en N pasos
radioReal = velocidadL/velocidadA; % radio de la trayectoria comandada
nPasos = 2*numerodepasos;          % dos vueltas

% Varianza del ruido del proceso (PROVISIONAL: sustituir por la calibracion de la odometria)
Qd = 1.1107e-04;
Qb = 2.9596e-05;
Qk_1 = [Qd 0; 0 Qb];

%% Posicion inicial
% Con velocidadA positiva (giro antihorario), desde el extremo derecho
% del circulo el robot debe empezar mirando hacia +Y.
Xini = [centroCirculo(1)+radioReal; centroCirculo(2); pi/2];

apoloPlaceMRobot(robot, [Xini(1) Xini(2) 0], Xini(3));
apoloResetOdometry(robot, Xini'); % la odometria arranca en la pose inicial
apoloUpdate();

Xk = Xini + [0.2; -0.2; 0.1];
Pxini = 0.05;  Pyini = 0.05;  Pthetaini = 0.01;
Pk = diag([Pxini Pyini Pthetaini]);

%% Modelo de observacion: balizas del supermercado
R_dist  = 1.6250e-04;    % varianza de la distancia medida [m^2]   (PROVISIONAL: calibrar el LMS100)
R_ang   = 2.9957e-04;    % varianza del angulo medido      [rad^2] (PROVISIONAL: calibrar el LMS100)
l_laser = 0.1;        % IMPORTANTE: comprobar desplazamiento en XML [m]
yaw_laser = 0;       % IMPORTANTE: comprobar orientacion del laser en XML [rad]
unidad_angulo = 'rad'; % cambiar a 'deg' si Apolo devuelve grados
idOffset = 0;        % 0 si los ID de Apolo empiezan en 1; 1 si empiezan en 0

% Fila i = [x y] de la baliza con mark_id i (supermercado_apolo.xml)
B = [
     2.640  11.290   % B001
     3.400  11.290   % B002
     2.640  14.660   % B003
     3.400  14.660   % B004
     5.890  11.290   % B005
     6.650  11.290   % B006
     5.890  14.660   % B007
     6.650  14.660   % B008
     9.140  11.290   % B009
     9.900  11.290   % B010
     9.140  14.660   % B011
     9.900  14.660   % B012
    12.390  11.290   % B013
    13.150  11.290   % B014
    12.390  14.660   % B015
    13.150  14.660   % B016
     2.640   6.190   % B017
     3.400   6.190   % B018
     2.640   9.560   % B019
     3.400   9.560   % B020
     5.890   6.190   % B021
     6.650   6.190   % B022
     5.890   9.560   % B023
     6.650   9.560   % B024
     9.140   6.190   % B025
     9.900   6.190   % B026
     9.140   9.560   % B027
     9.900   9.560   % B028
    12.390   6.190   % B029
    13.150   6.190   % B030
    12.390   9.560   % B031
    13.150   9.560   % B032
     3.840   2.440   % B033
     5.360   2.440   % B034
     3.840   3.200   % B035
     5.360   3.200   % B036
    10.640   2.440   % B037
    12.160   2.440   % B038
    10.640   3.200   % B039
    12.160   3.200   % B040
    % --- Balizas de pared (1 cm hacia dentro)
     0.010   2.000   % B041 pared O
     0.010   5.000   % B042 pared O
     0.010   8.000   % B043 pared O
     0.010  11.000   % B044 pared O
     0.010  14.000   % B045 pared O
    15.990   2.000   % B046 pared E
    15.990   5.000   % B047 pared E
    15.990   8.000   % B048 pared E
    15.990  11.000   % B049 pared E
    15.990  14.000   % B050 pared E
     2.000  16.990   % B051 pared N
     5.000  16.990   % B052 pared N
     8.000  16.990   % B053 pared N
    11.000  16.990   % B054 pared N
    14.000  16.990   % B055 pared N
     2.000   0.010   % B056 pared S
     5.000   0.010   % B057 pared S
    11.000   0.010   % B058 pared S
    14.000   0.010   % B059 pared S
];

%% Algoritmo
Xreal      = zeros(3,nPasos);
Xestimado  = zeros(3,nPasos);
Pacumulado = zeros(3,nPasos);
nVistas    = zeros(1,nPasos);
nUsadas    = zeros(1,nPasos);
nRechazadas = zeros(1,nPasos);
Xodometria = zeros(3,nPasos);
Xpredicho  = zeros(3,nPasos);
primerDiagnostico = false;

% IMPORTANTE: el resultado de Apolo suele ser un vector FILA 1x3.
% La muestra actual se convierte a columna en el bucle. Si la anterior
% queda como fila, MATLAB hace expansion implicita al restarlas (2x2),
% y el primer incremento de la prediccion es completamente incorrecto.
odo_prev = apoloGetOdometry(robot);
odo_prev = odo_prev(:);                    % FIX CLAVE: siempre 3x1
assert(numel(odo_prev)>=3 && all(isfinite(odo_prev(1:3))), ...
    'Odometria inicial de Apolo invalida');
odo_prev = odo_prev(1:3);

% Referencia odometrica independiente del EKF: "dead reckoning".
% Integramos distancia recorrida dd y giro dth, ambos obtenidos de Apolo,
% desde la pose inicial Xini. NUNCA usamos balizas ni pose real en Xodo.
% Esta forma no presupone que (odo(1),odo(2)) esten en ejes globales.
odo_referencia = odo_prev;  % solo para comprobar el primer incremento
Xodo = Xini;
fprintf('Odometria inicial: [%.3f, %.3f, %.3f]; pose inicial: [%.3f, %.3f, %.3f]\n', ...
        odo_referencia, Xini);

for l = 1:nPasos
    % Apolo mueve el robot (comando del controlador / teleoperacion)
    choque = false;
    for s = 1:subpasos
        if ~apoloMoveMRobot(robot, [velocidadL velocidadA], timestep/subpasos)
            choque = true;
            break
        end
    end
    apoloUpdate();
    if choque
        warning('El robot ha chocado en el paso %d. Se detiene la prueba.', l);
        nPasos = l-1;
        Xreal = Xreal(:,1:nPasos);
        Xestimado = Xestimado(:,1:nPasos);
        Pacumulado = Pacumulado(:,1:nPasos);
        nVistas = nVistas(1:nPasos);
        nUsadas = nUsadas(1:nPasos);
        nRechazadas = nRechazadas(1:nPasos);
        Xodometria = Xodometria(:,1:nPasos);
        Xpredicho = Xpredicho(:,1:nPasos);
        break
    end

    % Odometria de Apolo: incremento desde la lectura anterior
    odo = apoloGetOdometry(robot);
    odo = odo(:);
    assert(numel(odo)>=3 && all(isfinite(odo(1:3))), ...
        'Odometria de Apolo invalida en paso %d', l);
    odo = odo(1:3);
    dth = atan2(sin(odo(3)-odo_prev(3)), cos(odo(3)-odo_prev(3)));
    deltaOdo = odo(1:2)-odo_prev(1:2);   % 2x1: nunca matriz 2x2
    assert(isequal(size(deltaOdo),[2 1]), ...
        'El incremento de odometria debe ser 2x1');
    % El modulo no depende del marco angular de la odometria de Apolo.
    % Como esta trayectoria solo avanza hacia delante, el signo es conocido.
    % La version anterior proyectaba sobre el yaw odometrico, lo que
    % podia anular casi todo el desplazamiento si los marcos no coincidian.
    dd = sign(velocidadL) * hypot(deltaOdo(1), deltaOdo(2));
    if l <= 3
        fprintf('Paso %d: dodo=[%+.4f,%+.4f] m, dd=%+.4f m, dtheta=%+.4f rad\n', ...
            l,deltaOdo(1),deltaOdo(2),dd,dth);
    end
    if abs(dd) > 2.5*abs(velocidadL*timestep)+0.05
        error(['Incremento de odometria anomalo en paso %d: %.3f m. ' ...
               'Revisa apoloGetOdometry/reset y las dimensiones'],l,dd);
    end
    if l == 1 && abs(dd)>1e-9
        angOdoXY = atan2(deltaOdo(2),deltaOdo(1));
        angEsperado = atan2(sin(Xini(3)+dth/2),cos(Xini(3)+dth/2));
        diferenciaEjes = atan2(sin(angEsperado-angOdoXY),cos(angEsperado-angOdoXY));
        fprintf(['Primer paso: rumbo de delta XY crudo=%.3f rad, ' ...
                 'rumbo de avance global=%.3f rad, diferencia=%.3f rad (%.1f grados)\n'], ...
                 angOdoXY,angEsperado,diferenciaEjes,diferenciaEjes*180/pi);
    end
    odo_prev = odo;
    Uk = [dd; dth];

    % SOLO odometria (sin balizas, sin EKF): integra sus propios incrementos.
    theta_medio_odo = Xodo(3) + dth/2;
    Xodo(1:2) = Xodo(1:2) + dd*[cos(theta_medio_odo); sin(theta_medio_odo)];
    Xodo(3) = atan2(sin(Xodo(3)+dth),cos(Xodo(3)+dth));

    %Nuevo ciclo k-1 = k
    Xk_1 = Xk;
    Pk_1 = Pk;

    % Prediccion
    X_k = [(Xk_1(1) + Uk(1)*cos(Xk_1(3)+(Uk(2)/2)));
           (Xk_1(2) + Uk(1)*sin(Xk_1(3)+(Uk(2)/2)));
           atan2(sin(Xk_1(3)+Uk(2)), cos(Xk_1(3)+Uk(2)))];
    Ak = [1 0 (-Uk(1)*sin(Xk_1(3)+Uk(2)/2)); % matriz de transicion de estado
          0 1 ( Uk(1)*cos(Xk_1(3)+Uk(2)/2));
          0 0 1];
    Bk = [cos(Xk_1(3)+Uk(2)/2)  (-0.5*Uk(1)*sin(Xk_1(3)+Uk(2)/2)); % matriz de entrada del control
          sin(Xk_1(3)+Uk(2)/2)  ( 0.5*Uk(1)*cos(Xk_1(3)+Uk(2)/2));
          0                     1];
    P_k = Ak*Pk_1*Ak' + Bk*Qk_1*Bk';
    P_k = (P_k+P_k')/2;

    % Correccion con las balizas que ve el laser de Apolo
    lm = apoloGetLaserLandMarks(laser);
    
    if isempty(lm)
        lm = struct('id', [], 'angle', [], 'distance', []);
    end

    % Si el mark_id de Apolo es 0..58, poner idOffset=1 al inicio.
    % NO corregir IDs sin verificar en el XML.
    if idOffset ~= 0
        if numel(lm)==1
            lm.id = lm.id + idOffset;
        else
            for j = 1:numel(lm)
                lm(j).id = lm(j).id + idOffset;
            end
        end
    end

    [Xk, Pk, info] = ekf_update_corregido(X_k, P_k, lm, B, ...
                         R_dist, R_ang, l_laser, yaw_laser, unidad_angulo);

    % Muestra una medida real frente a la prevista para detectar problemas
    % de unidades, orientacion del laser o correspondencia ID <-> mapa.
    if ~primerDiagnostico && ~isempty(info.muestra)
        v = info.muestra;
        fprintf(['Primer ID %d: distancia medida=%.3f m, prevista=%.3f m; ' ...
                 'angulo medido=%.3f rad, previsto=%.3f rad; NIS=%.2f\n'], ...
                 v.id, v.z(1), v.h(1), v.z(2), ...
                 atan2(sin(v.h(2)),cos(v.h(2))), v.nis);
        primerDiagnostico = true;
    end
    if info.nInvalidas>0 && (l==1 || mod(l,10)==0)
        warning('Paso %d: %d lecturas con ID invalido, distancia <= 0 o valores no finitos.', ...
                l, info.nInvalidas);
    end
    if info.angulosFueraDeRango && l==1
        warning('Hay angulos que superan 2*pi: comprueba unidad_angulo.');
    end
    if info.nValidas>0 && info.nUsadas==0 && (l==1 || mod(l,10)==0)
        warning('Paso %d: ninguna de %d medidas aceptada. Revisa IDs, XML y unidades.', ...
                l, info.nValidas);
    end

    % Solo para las graficas
    p = apoloGetLocationMRobot(robot);
    Xreal(:,l)      = [p(1); p(2); p(4)];
    Xestimado(:,l)  = Xk;
    Xpredicho(:,l)  = X_k;
    % Trayectoria odometrica independiente, expresada en marco global
    % a partir de Xini. No contiene ninguna correccion con balizas.
    Xodometria(:,l) = Xodo;
    Pacumulado(:,l) = diag(Pk);
    nVistas(l)      = info.nDetectadas;
    nUsadas(l)      = info.nUsadas;
    nRechazadas(l)  = info.nRechazadas;
end

%% Representacion grafica
ang = @(a) atan2(sin(a), cos(a));
t   = 1:nPasos;
obst = [ 2.65  3.39 11.30 14.65;   5.90  6.64 11.30 14.65;   9.15  9.89 11.30 14.65;  12.40 13.14 11.30 14.65;
         2.65  3.39  6.20  9.55;   5.90  6.64  6.20  9.55;   9.15  9.89  6.20  9.55;  12.40 13.14  6.20  9.55;
         3.85  5.35  2.45  3.19;  10.65 12.15  2.45  3.19 ];

figure(1);
clf(1);  % Borra las trayectorias de ejecuciones anteriores

subplot(2,2,1);
hold on;
axis equal;
axis([0 16 0 17]); axis equal; axis([0 16 0 17]);
for o = 1:size(obst,1)
    rectangle('Position', [obst(o,1) obst(o,3) obst(o,2)-obst(o,1) obst(o,4)-obst(o,3)], ...
              'FaceColor', [0.85 0.85 0.85], 'EdgeColor', [0.5 0.5 0.5]);
end
hR = plot(Xreal(1,:), Xreal(2,:), '.k');
hE = plot(Xestimado(1,:), Xestimado(2,:), '.r');
hO = plot(Xodometria(1,:), Xodometria(2,:), ':', 'Color', [0 0.5 0]);
hB = plot(B(:,1), B(:,2), 'sb', 'MarkerSize', 4);
legend([hR hE hO hB], 'Movimiento real (Apolo)', 'EKF', 'Odometria integrada', 'Balizas', ...
       'Location', 'southoutside');
xlabel('X (m)'); ylabel('Y (m)');

subplot(2,2,2); plot(t, Xreal(1,:), 'k', t, Xestimado(1,:), 'g');
xlabel('t (muestras)'); ylabel('X (m)');
subplot(2,2,3); plot(t, Xreal(2,:), 'k', t, Xestimado(2,:), 'g');
xlabel('t (muestras)'); ylabel('Y (m)');
subplot(2,2,4); plot(t, ang(Xreal(3,:)), 'k', t, Xestimado(3,:), 'g');
xlabel('t (muestras)'); ylabel('\theta (rad)');

% Error con bandas de +-3 sigma: si el error se sale a menudo, Q o R estan mal calibradas
figure(2);
err = [Xestimado(1,:)-Xreal(1,:); Xestimado(2,:)-Xreal(2,:); ang(Xestimado(3,:)-Xreal(3,:))];
nombres = {'Error X (m)', 'Error Y (m)', 'Error \theta (rad)'};
for i = 1:3
    subplot(4,1,i); hold on;
    plot(t, err(i,:), 'b');
    plot(t,  3*sqrt(Pacumulado(i,:)), 'r--', t, -3*sqrt(Pacumulado(i,:)), 'r--');
    ylabel(nombres{i});
end
subplot(4,1,4);
stairs(t, nVistas, 'k'); hold on;
stairs(t, nUsadas, 'g');
stairs(t, nRechazadas, 'r');
legend('Detectadas', 'Usadas', 'Rechazadas');
xlabel('t (muestras)'); ylabel('Balizas');

figure(3);
nombresP = {'Varianza X (m^2)', 'Varianza Y (m^2)', 'Varianza \theta (rad^2)'};
for i = 1:3
    subplot(3,1,i); plot(t, Pacumulado(i,:), 'b'); ylabel(nombresP{i});
end
xlabel('t (muestras)');

%% Diagnostico cuantitativo
if nPasos > 0
    errOdo = [Xodometria(1,:)-Xreal(1,:); ...
              Xodometria(2,:)-Xreal(2,:); ...
              ang(Xodometria(3,:)-Xreal(3,:))];
    rmseEKF = sqrt(mean(err.^2,2));
    rmseOdo = sqrt(mean(errOdo.^2,2));
    fprintf('RMSE EKF: x=%.4f m, y=%.4f m, theta=%.4f rad\n', rmseEKF);
    fprintf('RMSE odo: x=%.4f m, y=%.4f m, theta=%.4f rad\n', rmseOdo);
    fprintf('Balizas: detectadas=%d, usadas=%d, rechazadas=%d\n', ...
             sum(nVistas), sum(nUsadas), sum(nRechazadas));
end
