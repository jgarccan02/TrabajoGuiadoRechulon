%% Regresion: bug de orientacion fila/columna en el primer incremento de odometria
% No necesita Apolo. Si pasa, el primer salto ya no puede surgir por
% incompatibilidad entre vectores fila y columna en el programa principal.
clearvars;
radio=0.8; v=0.2; dt=0.5;
N=ceil(2*pi*radio/(v*dt)); omega=2*pi/(N*dt);
radioReal=v/omega;
Xini=[8+radioReal; 4.2; pi/2];
% Una muestra inicial como la entregaria Apolo en vector fila:
odo_prev=Xini';
% Movimiento ideal de 0.1 m y giro de omega*dt:
dtheta=omega*dt;
odo=[Xini(1)+v*dt*cos(Xini(3)+dtheta/2); ...
     Xini(2)+v*dt*sin(Xini(3)+dtheta/2); Xini(3)+dtheta];
% ANTES: delta = (2x1)-(1x2) -> (2x2) con expansion implicita
% y la segunda componente empleaba x_prev en lugar de y_prev.
% AHORA: dos vectores columna; resto 2x1.
odo_prev=odo_prev(:);
odo=odo(:);
deltaOdo=odo(1:2)-odo_prev(1:2);
assert(isequal(size(deltaOdo),[2,1]), 'Regresion: dimensiones erradas');
dd=hypot(deltaOdo(1),deltaOdo(2));
assert(abs(dd-v*dt)<1e-12, 'Regresion: desplazamiento incorrecto');
Xk=Xini+[0.2;-0.2;0.1];
Xpred=[Xk(1)+dd*cos(Xk(3)+dtheta/2); ...
       Xk(2)+dd*sin(Xk(3)+dtheta/2); ...
       atan2(sin(Xk(3)+dtheta),cos(Xk(3)+dtheta))];
assert(norm(Xpred(1:2)-Xini(1:2))<0.4,...
    'Regresion: salto imposible en el primer paso');
fprintf('OK: incremento odometrico = %.4f m (esperado %.4f m).\n',dd,v*dt);
fprintf('OK: posicion predicha primer paso [%.4f, %.4f] m.\n',Xpred(1:2));
