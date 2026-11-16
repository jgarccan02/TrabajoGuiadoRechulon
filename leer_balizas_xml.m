function B = leer_balizas_xml(archivoXML)
% Lee las coordenadas 2D de LandMark en el mundo de Apolo.
% La fila i de B corresponde exactamente a mark_id=i.
if exist(archivoXML,'file')~=2
    error('No existe el XML: %s',archivoXML);
end
try
    doc = xmlread(archivoXML);
catch ME
    error('No se pudo leer el XML de Apolo: %s', ME.message);
end
nodos = doc.getElementsByTagName('LandMark');
n = nodos.getLength();
if n==0
    error('El XML no tiene elementos LandMark.');
end
ids = zeros(n,1);
coords = zeros(n,2);
for k=1:n
    nodo = nodos.item(k-1);
    ids(k) = str2double(char(nodo.getAttribute('mark_id')));
    posiciones = nodo.getElementsByTagName('position');
    if posiciones.getLength()==0
        error('Falta position en la baliza numero %d',k);
    end
    texto = char(posiciones.item(0).getTextContent());
    pos = sscanf(regexprep(texto,'[{}]',''), '%f,%f,%f');
    if numel(pos)~=3
        error('Posicion incorrecta en la baliza numero %d: %s',k,texto);
    end
    coords(k,:) = pos(1:2)';
end
if any(~isfinite(ids) | ids<1 | ids~=fix(ids)) || numel(unique(ids))~=n
    error('Los mark_id del XML no son enteros positivos y unicos.');
end
B = nan(max(ids),2);
B(ids,:) = coords;
if any(~isfinite(B(:)))
    error('Los mark_id del XML no son consecutivos desde 1.');
end
fprintf('Mapa del XML: %d balizas; IDs del 1 al %d.\n', n, size(B,1));
end
