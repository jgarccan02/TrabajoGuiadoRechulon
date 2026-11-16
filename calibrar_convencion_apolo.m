function [modelo, informe] = calibrar_convencion_apolo(scans, poses, B, l_laser)
% Determina el formato de las medidas de apoloGetLaserLandMarks.
% scans: celda con lecturas desde dos poses CONOCIDAS (robot colocado alli).
% poses: matriz N x 3, [x y theta] correspondiente a cada lectura.
% Usa SOLO las poses fijadas con apoloPlaceMRobot, NO la pose real medida.
% No toca el filtro; el modelo seleccionado se utiliza en cada lectura.
%
% Transformacion de lecturas:
%   distancia_m = escalaDist * distance
%   angulo_rad  = wrap(signo * escalaAng * angle + offset)
%   referencia  = 'rel' (angulo del robot) o 'abs' (angulo global)

if ~iscell(scans) || size(poses,1)~=numel(scans) || size(poses,2)~=3
    error('scans y poses deben contener las mismas poses [x y theta].');
end
ids = [];
draw = [];
araw = [];
dEsperada = [];
phiMundo = [];
phiRobot = [];
porPose = zeros(numel(scans),1);
for k=1:numel(scans)
    lm = scans{k};
    if isempty(lm), continue; end
    if ~isstruct(lm) || ~all(isfield(lm,{'id','distance','angle'}))
        error('La estructura de LandMarks no tiene id, distance, angle. Usa disp(lm).');
    end
    if numel(lm)==1
        i = double(lm.id(:)); d = double(lm.distance(:)); a = double(lm.angle(:));
    else
        i = double([lm.id]'); d = double([lm.distance]'); a = double([lm.angle]');
        i = i(:); d = d(:); a = a(:);
    end
    if numel(i)~=numel(d) || numel(i)~=numel(a)
        error('Las longitudes id/distance/angle no coinciden.');
    end
    ok = isfinite(i) & i==fix(i) & i>=1 & i<=size(B,1) & ...
         isfinite(d) & d>0 & isfinite(a);
    i=i(ok); d=d(ok); a=a(ok);
    porPose(k)=numel(i);
    if isempty(i), continue; end
    th = poses(k,3);
    xs = poses(k,1)+l_laser*cos(th);
    ys = poses(k,2)+l_laser*sin(th);
    dx = B(i,1)-xs;
    dy = B(i,2)-ys;
    ids = [ids; i]; %#ok<AGROW>
    draw = [draw; d]; %#ok<AGROW>
    araw = [araw; a]; %#ok<AGROW>
    dEsperada = [dEsperada; hypot(dx,dy)]; %#ok<AGROW>
    phiMundo = [phiMundo; atan2(dy,dx)]; %#ok<AGROW>
    phiRobot = [phiRobot; atan2(sin(atan2(dy,dx)-th),cos(atan2(dy,dx)-th))]; %#ok<AGROW>
end

fprintf('Diagnostico inicial: %s medidas validas en %d poses.\n', ...
        mat2str(porPose'),numel(scans));
if numel(ids)<2 || any(porPose==0)
    error(['Faltan medidas validas en alguna de las dos orientaciones. ' ...
           'No es seguro identificar la referencia angular con una sola pose. ' ...
           'Comprueba el XML abierto, los IDs y el campo angle.']);
end

% Distancia: Apolo puede entregar m, cm o mm. No asumirlo sin verificar.
escalasD = [1, 0.01, 0.001];
errorDist = zeros(numel(escalasD),1);
for j=1:numel(escalasD)
    e = escalasD(j)*draw-dEsperada;
    errorDist(j) = sqrt(mean(e.^2));
end
[rmseDist, jMejor] = min(errorDist);
escalaDist = escalasD(jMejor);

% Angulo: comprobar rad/deg, sentido del giro, referencia y offset.
% Se usan dos yaw diferentes: con un unico yaw referencia global/local
% seria ambiguo si se admite un offset angular.
escalasA = [1, pi/180];
unidades = {'rad','deg'};
referencias = {'rel','abs'};
offsets = [0,pi/2,-pi/2,pi];
C = struct('rmse',{},'escalaAng',{},'unidad',{},'signo',{}, ...
           'referencia',{},'offset',{});
for iu=1:2
  for sg=[1 -1]
    for ir=1:2
      if ir==1, pred=phiRobot; else, pred=phiMundo; end
      for io=1:numel(offsets)
        phi=sg*escalasA(iu)*araw+offsets(io);
        e=atan2(sin(phi-pred),cos(phi-pred));
        v.rmse=sqrt(mean(e.^2));
        v.escalaAng=escalasA(iu);
        v.unidad=unidades{iu};
        v.signo=sg;
        v.referencia=referencias{ir};
        v.offset=offsets(io);
        C(end+1)=v; %#ok<AGROW>
      end
    end
  end
end
puntajes = [C.rmse];
[rmseAng,idx] = min(puntajes);
sel=C(idx);

fprintf('Mejor escala de distancia: %.6g (RMSE %.3f m).\n',escalaDist,rmseDist);
fprintf('Angulos: %s, signo %+d, referencia %s, offset %.4f rad (RMSE %.4f rad).\n', ...
        sel.unidad,sel.signo,sel.referencia,sel.offset,rmseAng);
[~,ord]=sort(puntajes);
for j=1:min(3,numel(ord))
   c=C(ord(j));
   fprintf(' Candidato %d: %s, signo %+d, %s, offset %+.3f, RMSE %.4f rad\n', ...
           j,c.unidad,c.signo,c.referencia,c.offset,c.rmse);
end
fprintf(['Ejemplo: ID %d, distancia cruda %.4f, predicha %.4f m; ' ...
         'angulo crudo %.4f.\n'],ids(1),draw(1),dEsperada(1),araw(1));

informe=struct('nMedidas',numel(ids),'porPose',porPose, ...
               'rmseDist',rmseDist,'rmseAng',rmseAng, ...
               'idEjemplo',ids(1),'dCrudaEjemplo',draw(1), ...
               'dPrevistaEjemplo',dEsperada(1),'aCrudaEjemplo',araw(1));

if rmseDist>0.4 || rmseAng>0.25
    error(['Las medidas NO concuerdan con las balizas del XML. ' ...
           'Comprueba el mundo realmente abierto en Apolo y la salida de ' ...
           'apoloGetLaserLandMarks. No se desactiva el NIS.']);
end

modelo=struct('escalaDist',escalaDist, ...
              'escalaAng',sel.escalaAng,'signo',sel.signo, ...
              'offset',sel.offset,'referencia',sel.referencia, ...
              'unidadOriginal',sel.unidad);
end
