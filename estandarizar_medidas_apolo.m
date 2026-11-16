function lm = estandarizar_medidas_apolo(lm, modelo)
% Transforma distance y angle a metros y radianes segun calibracion.
% La referencia rel/abs NO se cambia aqui; la maneja el EKF en su h y H.
if isempty(lm), return; end
if ~isstruct(lm) || ~all(isfield(lm,{'id','distance','angle'}))
    error('La estructura de LandMarks no contiene id/distance/angle.');
end
for k=1:numel(lm)
    lm(k).distance=double(lm(k).distance)*modelo.escalaDist;
    a=modelo.signo*modelo.escalaAng*double(lm(k).angle)+modelo.offset;
    lm(k).angle=atan2(sin(a),cos(a));
end
end
