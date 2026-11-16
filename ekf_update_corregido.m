function [X, P, info] = ekf_update_corregido(X, P, lm, B, R_dist, R_ang, l_laser, yaw_laser, unidad_angulo)
% EKF de localizacion 2D con balizas conocidas (distancia + angulo).
%
% X = [x; y; theta] en m, m, rad; P = covarianza 3x3.
% B(i,:) = [x_i y_i] corresponde al ID i de Apolo.
% l_laser: desplazamiento del laser sobre el eje +X del robot [m].
% yaw_laser: giro del eje del laser respecto al frente del robot [rad].
% unidad_angulo: 'rad' (por defecto) o 'deg'.
%
% El Jacobiano se calcula para el desplazamiento del laser. La actualizacion
% es secuencial y rechaza observaciones incompatibles con un test NIS.
%
% Guardar con el nombre ekf_update_corregido.m (o cambiar funcion/archivo a ekf_update).

if nargin < 7 || isempty(l_laser), l_laser = 0.1; end
if nargin < 8 || isempty(yaw_laser), yaw_laser = 0; end
if nargin < 9 || isempty(unidad_angulo), unidad_angulo = 'rad'; end

X = X(:);
P = (P + P') / 2;
assert(numel(X)==3 && isequal(size(P),[3 3]), 'Estado o covarianza de dimensiones incorrectas.');
assert(R_dist>0 && R_ang>0, 'R_dist y R_ang deben ser varianzas positivas.');
assert(size(B,2)==2, 'B debe ser una matriz N x 2.');

info = struct('nDetectadas',0, 'nValidas',0, 'nUsadas',0, ...
              'nRechazadas',0, 'nInvalidas',0, ...
              'ids',[], 'idsRechazadas',[], 'nis',[], ...
              'innov',zeros(2,0), 'muestra',[], 'angulosFueraDeRango',false);

if isempty(lm), return; end
if ~isstruct(lm) || ~all(isfield(lm, {'id','distance','angle'}))
    error('Las balizas deben contener los campos id, distance y angle. Comprueba disp(lm).');
end

% Admite struct escalar con campos vector o array de structs (una por baliza).
if numel(lm)==1
    ids = lm.id(:);
    d = lm.distance(:);
    a = lm.angle(:);
else
    ids = [lm.id]';
    d = [lm.distance]';
    a = [lm.angle]';
end
ids = ids(:); d = d(:); a = a(:);
assert(numel(ids)==numel(d) && numel(ids)==numel(a), ...
       'Los arrays id, distance y angle tienen distinto numero de elementos.');
info.nDetectadas = numel(ids);

switch lower(char(unidad_angulo))
    case 'rad'
        info.angulosFueraDeRango = any(isfinite(a) & abs(a)>2*pi+0.01);
    case 'deg'
        a = a*pi/180;
    otherwise
        error('unidad_angulo debe ser rad o deg.');
end

% Los IDs se consideran 1..size(B,1); NO se corrigen automaticamente.
valida = isfinite(ids) & ids==fix(ids) & ids>=1 & ids<=size(B,1) & ...
         isfinite(d) & d>0 & isfinite(a);
info.nInvalidas = sum(~valida);
ids = ids(valida); d = d(valida); a = a(valida);
if isempty(ids), return; end

% Evita utilizar dos veces un mismo ID en el mismo ciclo.
[ids, idx] = unique(ids, 'stable');
d = d(idx); a = a(idx);
info.nValidas = numel(ids);

R = diag([R_dist, R_ang]);
umbral_nis = 13.8155;       % chi-cuadrado 2 DOF, nivel 99.9 %
I = eye(3);

for k = 1:numel(ids)
    theta = X(3);
    c = cos(theta); s = sin(theta);
    xs = X(1) + l_laser*c;
    ys = X(2) + l_laser*s;
    dx = B(ids(k),1) - xs;
    dy = B(ids(k),2) - ys;
    q = dx*dx + dy*dy;
    if q < 1e-10
        info.idsRechazadas(end+1,1) = ids(k);
        continue;
    end
    r = sqrt(q);
    h = [r; atan2(dy,dx) - theta - yaw_laser];
    z = [d(k); a(k)];
    H = [ -dx/r, -dy/r,  l_laser*(dx*s-dy*c)/r; ...
           dy/q, -dx/q, -1-l_laser*(dx*c+dy*s)/q ];

    nu = z - h;
    nu(2) = atan2(sin(nu(2)), cos(nu(2)));
    S = H*P*H' + R;
    nis = nu'*(S\nu);
    info.nis(end+1,1) = nis;
    if isempty(info.muestra)
        info.muestra = struct('id',ids(k), 'z',z, 'h',h, 'innov',nu, 'nis',nis);
    end

    if ~isfinite(nis) || nis > umbral_nis
        info.idsRechazadas(end+1,1) = ids(k);
        continue;
    end

    % EKF secuencial: se recalcula h y H tras cada correccion aceptada.
    K = (P*H')/S;
    X = X + K*nu;
    X(3) = atan2(sin(X(3)), cos(X(3)));
    J = I-K*H;
    P = J*P*J' + K*R*K';   % Forma de Joseph
    P = (P+P')/2;
    info.ids(end+1,1) = ids(k);
    info.innov(:,end+1) = nu;
end

info.nUsadas = numel(info.ids);
info.nRechazadas = numel(info.idsRechazadas);
end
