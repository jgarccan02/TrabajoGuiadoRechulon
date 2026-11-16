%% Prueba sintetica de identificacion del formato del laser y del EKF.
% NO necesita Apolo. Ejecutar con estos archivos .m en el path de MATLAB.
clearvars;
B=[2 3; 7 8; 9 2; 11 10; 1 10; 4 1; 14 7];
poses=[6 5 0.3; 6 5 0.9];
l=0.1;
referencias={'rel','abs'};
for kref=1:2
  scans=cell(2,1);
  for k=1:2
    th=poses(k,3);
    dx=B(:,1)-poses(k,1)-l*cos(th);
    dy=B(:,2)-poses(k,2)-l*sin(th);
    d=hypot(dx,dy);
    if kref==1
        ang=atan2(sin(atan2(dy,dx)-th),cos(atan2(dy,dx)-th));
    else
        ang=atan2(dy,dx);
    end
    % Apolo hipotetico que da cm, grados, angulo horario, offset de pi/2.
    ang_crudo=-(ang-pi/2)*180/pi;
    scans{k}=struct('id',(1:size(B,1))', 'distance',d*100, 'angle',ang_crudo);
  end
  [modSensor,infCal]=calibrar_convencion_apolo(scans,poses,B,l);
  assert(strcmp(modSensor.referencia,referencias{kref}), ...
         'No se identifico la referencia angular correcta');
  assert(abs(modSensor.escalaDist-0.01)<eps, 'Error en la escala de distancia');
  assert(infCal.rmseAng<1e-8 && infCal.rmseDist<1e-8, ...
         'La calibracion no coincide con la simulacion ideal');
  lm=estandarizar_medidas_apolo(scans{1},modSensor);
  Xreal=poses(1,:)';
  X=Xreal+[0.2;-0.2;0.1];
  P=diag([0.05 0.05 0.01]);
  for j=1:4
    [X,P,info]=ekf_update_corregido(X,P,lm,B,1.625e-4,2.9957e-4,l,0,'rad',modSensor.referencia);
  end
  assert(info.nUsadas>0,'No se aceptaron observaciones simuladas');
  err=X-Xreal;
  err(3)=atan2(sin(err(3)),cos(err(3)));
  assert(norm(err(1:2))<0.02,'El EKF no localiza correctamente la posicion simulada');
  if kref==1
      assert(abs(err(3))<0.02,'El EKF relativo no estima bien la orientacion');
  end
end
disp('Prueba sintetica de medidas relativas/absolutas completada.');
