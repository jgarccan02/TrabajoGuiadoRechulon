# Robot guía de supermercado — Guiado y Navegación de Robots 2026/27

Simulación en Apolo controlada desde MATLAB.

**Entrega:** 12 de enero de 2027, 15:30, en Moodle (un único ZIP o RAR).

---

## Estado y lo que queda por hacer

- [x] Mapa con balizas y modelo de observación distinto al del ejemplo (distancia + ángulo).
- [x] EKF conectado a Apolo.
- [x] Calibración del láser: R_dist = 1,6·10⁻⁴ m², R_ang = 3,0·10⁻⁴ rad² (comprobado con la tabla medida/calculada desde la pose real).
- [x] Verificar el EKF en Apolo. Hay que confirmar que la estimación sigue a la trayectoria real y que el error queda dentro de las bandas ±3σ.
- [ ] **Dicho por claude ->** Revisar las unidades de Q. `calibrasiondeq.m` calcula la varianza de la velocidad ((m/s)²) en pasos de 0,1 s, pero Q tiene que ser la varianza del *incremento* en cada ciclo del filtro (m² y rad² cada 0,5 s). Tal como está, Q sale unas 20 veces más grande de lo que toca.
- [ ] **Dicho por claude ->** Calibración de odometría completa: tramos rectos de varias longitudes y giros de varios ángulos por separado, para ajustar `Qd = k_d·|Δd|` y `Qb = k_θ·|Δθ|`. Comprobar que la media del error es ≈ 0.
- [ ] Experimentos para la memoria, cambiando un parámetro cada vez y con gráficas (lo pide en el guión del trabajo, *"analizar la influencia de los principales parámetros del filtro (P(0|0), Q(k), R(k)), mostrando gráficamente la evolución de aquellos que considere más significativos."*, *"Experimentar con el algoritmo y analizar la influencia de los principales parámetros del filtro"* y *"análisis de la influencia del mapa y de los sensores escogidos en la estimación del estado"*):
  - P(0|0) pequeña con la pose inicial mal frente a P(0|0) coherente.
  - Q ×10 y ÷10; R ×10 y ÷10.
  - Quitar balizas (solo paredes, solo estanterías) y pasar por un pasillo largo: influencia del mapa.
- [ ] Pasar el EKF a una función (`[X,P] = localizacion_step(...)`) para usarlo en el demostrador.

- [ ] Controlador básico que lleve el robot por una secuencia de puntos usando la `X` del EKF.
- [ ] Control reactivo con el LMS100 en modo nube de puntos (`apoloGetLaserData`) para esquivar personas y obstáculos que no están en el mapa y pasar por pasos estrechos.
- [ ] Experimentos de ajuste de ganancias y umbrales.

- [ ] Definición de la posición de los diferentes objetos en las estanterias.
- [ ] Planificador offline que genere la trayectoria (lista de puntos) en el mapa, de la entrada a la estantería del producto pedido.
- [ ] Integrarlo con el controlador.
- [ ] Pruebas con varios destinos.

- [ ] Demostrador (`demostrador.m`): se pide un producto → el planificador da la ruta → el controlador la sigue usando la pose del EKF → el reactivo esquiva lo imprevisto.
- [ ] Memoria en PDF (no más de 20 páginas): aplicación; calibración de la locomoción; sensores y su calibración; localización (modelos, Φ(k), H(k) y experimentos con P(0|0), Q y R); control reactivo; planificación; demostrador. Incluir el reparto de roles.
- [ ] Vídeo del demostrador.
- [ ] Subirlo a Moodle.

---

## Contenido del repositorio

| Fichero | Qué es |
|---|---|
| `supermercado_apolo.xml` | Mundo de Apolo. Planta de 16 × 17 m con entrada en la pared sur (x = 7–9 m), 8 estanterías (0,74 × 3,35 × 2 m), 2 cajas de cobro (1,5 × 0,74 × 1,18 m), el robot en (8; 1,1) mirando hacia +y, el láser LMS100 10 cm por delante del robot a 0,4 m de altura y 59 balizas: B001–B040 en las esquinas de estanterías y cajas y B041–B059 en las paredes. |
| `kalman_supermercado_corregido.m` | Script principal de localización. EKF conectado a Apolo: el robot da dos vueltas a un círculo, predice con la odometría de Apolo y corrige con las balizas que ve el láser. Dibuja la trayectoria real, la del EKF y la de solo odometría, el error con bandas ±3σ, las balizas detectadas/usadas/rechazadas y la evolución de P. Al final imprime el RMSE del EKF y el de la odometría sola. |
| `ekf_update_corregido.m` | Función de corrección del EKF. Modelo de observación distancia + ángulo a cada baliza vista, con el láser desplazado 10 cm. Corrección secuencial baliza a baliza, descarte de medidas incompatibles (test NIS, χ² con 2 g.d.l. al 99,9 %) y forma de Joseph para P. |
| `calibrasiondeq.m` | Calibración de la odometría (para Q): compara los incrementos de la odometría con los de la pose real de Apolo en 100 pasos y saca media y varianza del error. |
| `calibrasionder.m` | Calibración del láser (para R): robot quieto, 100 lecturas de una baliza, media y varianza del error en distancia y ángulo. |
| `LEEME_revision_v5.txt` | Notas hechas por chati que se tienen que revisar |

---

## Cómo ejecutar

1. Abrir `supermercado_apolo.xml` en Apolo.
2. Poner en el path de MATLAB la carpeta `Matlab` de Apolo (la ruta está al principio de cada script: cada uno debe poner la suya).
3. Ejecutar `kalman_supermercado_corregido.m` (con `ekf_update_corregido.m` en la misma carpeta).

Funciones de Apolo usadas (ver `ApoloPort.pdf`): `apoloPlaceMRobot`, `apoloResetOdometry`, `apoloMoveMRobot`, `apoloUpdate`, `apoloGetOdometry`, `apoloGetLaserLandMarks`, `apoloGetLocationMRobot`.

---

## Convenios

- **Marco global:** x de izquierda a derecha, y desde la entrada hacia el fondo, metros. θ en radianes, en [−π, π], 0 = mirando a +x.
- **Pose del robot:** `X = [x; y; theta]` (3×1). Su incertidumbre, `P` (3×3).
- **Mapa de balizas:** matriz `B` de 59×2. La fila *i* es la baliza con `mark_id` *i* del xml. *Si se mueve o se añade una baliza en el xml, hay que cambiarla también en `B`.*
- **Pose real** (`apoloGetLocationMRobot`): solo para calibrar y para las gráficas. Ni el filtro, ni el controlador, ni el planificador pueden usarla.
- **Controlador y planificador:** trabajan con la `X` que da el EKF.

---

## Evaluación

Localización y calibración 50 %, control reactivo 20 %, planificación 20 %, reparto de roles 10 %. Se valoran la originalidad, la corrección del planteamiento, la calidad de los experimentos y la claridad de la memoria.

---

***Tonto el que lo lea.***
