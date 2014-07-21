%% Ejemplo 2 Proyección Perspectiva


close all
clear all
clc

% Coordenadas del Centro de visión (cámara)
C = [0,0,0,1];
C1 = [0,0,1,1];
plot3([C(1) C1(1)],[C(2) C1(2)],[C(3) C1(3)]);
scatter3(C(1),C(2),C(3),'rx','LineWidth',4);
axis equal
hold on;

% Plano de proyección Z = 1;
Plano = [0,2,-1,1];
[PX,PY] = meshgrid(linspace(-1,1,50),linspace(-1,1,50));
PZ = -(Plano(1)*PX + Plano(2)*PY + Plano(4))/Plano(3);
surf(PX,PY,PZ,0.5*ones(size(PZ)),'EdgeAlpha',0,'FaceAlpha',0.1,'FaceColor','b');

% Supongamos una figura
Z_image = 2;
N = 100;
t = linspace(5,25,N);
X = 10*cos(t)./(2*t);
Y = 10*sin(t)./(2*t);
Z = Z_image*ones(size(X));
espiral = [X;Y;Z;ones(size(X))];
plot3(espiral(1,:),espiral(2,:),espiral(3,:),'LineWidth',2);
grid on

% Matriz de proyección
fx = 1;
fy = 1;
Skew = 0;
T = [fx Skew C(1) 0;0 fy C(2) 0; Plano];
T = [1 0 0 0; 0 1 0 0; 0 0 0 0; Plano];

espiralProyeccion2D = T*espiral;
espiralProyeccion3D = -espiralProyeccion2D./espiralProyeccion2D(4);
espiralProyeccion3D(3,:) = -(Plano(1)*espiralProyeccion3D(1,:) + Plano(2)*espiralProyeccion3D(2,:) + Plano(4))/Plano(3);

plot3(espiralProyeccion3D(1,:),espiralProyeccion3D(2,:),espiralProyeccion3D(3,:),'r','LineWidth',2);


