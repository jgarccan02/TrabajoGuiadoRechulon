%% PRUEBA SIN APOLO: desplazamientos XY odometricos en ejes rotados
% Genera una trayectoria circular ideal, construye un odometro simulado
% que acumula deltas en ejes -90 grados respecto a los globales y muestra
% que dibujar (x,y) crudos desplaza el circulo aunque el origen coincida.
clearvars;
v = 0.2; dt = 0.5; r = 0.8;
N = ceil(2*pi*r/(v*dt)); omega = 2*pi/(N*dt); nPasos=2*N;
Xini = [8 + v/omega; 4.2; pi/2];
Xreal = Xini; Xodo = Xini; odo = Xini;
trayReal=zeros(3,nPasos); trayCruda=zeros(3,nPasos);
trayIntegrada=zeros(3,nPasos);
Rmenos90 = [0 1;-1 0];
for k=1:nPasos
    dd=v*dt; dth=omega*dt;
    delta = dd*[cos(Xreal(3)+dth/2);sin(Xreal(3)+dth/2)];
    Xreal = Xreal + [delta;dth];
    Xreal(3) = atan2(sin(Xreal(3)),cos(Xreal(3)));
    odo(1:2)=odo(1:2)+Rmenos90*delta; % ejes locales del odometro
    odo(3)=atan2(sin(odo(3)+dth),cos(odo(3)+dth));
    Xodo(1:2)=Xodo(1:2)+dd*[cos(Xodo(3)+dth/2);sin(Xodo(3)+dth/2)];
    Xodo(3)=atan2(sin(Xodo(3)+dth),cos(Xodo(3)+dth));
    trayReal(:,k)=Xreal; trayCruda(:,k)=odo; trayIntegrada(:,k)=Xodo;
end
errCrudo=max(vecnorm(trayCruda(1:2,:)-trayReal(1:2,:)));
errIntegrado=max(vecnorm(trayIntegrada(1:2,:)-trayReal(1:2,:)));
assert(errCrudo>0.5,'La prueba no genero desfase de ejes');
assert(errIntegrado<1e-10,'La integracion odometrica deberia coincidir');
fprintf('OK: error maximo XY crudo = %.4f m; integrado = %.4g m.\n', ...
    errCrudo,errIntegrado);
figure; hold on; axis equal; grid on;
plot(trayReal(1,:),trayReal(2,:),'k','LineWidth',1.5);
plot(trayCruda(1,:),trayCruda(2,:),'g--');
plot(trayIntegrada(1,:),trayIntegrada(2,:),'r:','LineWidth',1.8);
legend('Real','Odometria XY en ejes girados','Odometria integrada');
xlabel('X (m)'); ylabel('Y (m)');
title('Prueba sintetica: el inicio coincide, los ejes de desplazamiento no');
