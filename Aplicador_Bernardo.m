%% Solver para el rigid body de los aplicadores
close all
clear all
clc
Npuntos =  4;
Puntos_I = [70.77 0 226.13;...
            70.77 37.33 285.35;...
            70.77 -38.18 307.63;...
            70.77 21.38 332.85;...
                                ];
Origen = [0 0 0];



% figure
% scatter3(Puntos_I(:,1),Puntos_I(:,2),Puntos_I(:,3),'o','LineWidth',3);
% hold on
% scatter3(Origen(:,1),Origen(:,2),Origen(:,3),'ok','LineWidth',3)
% Definición del plano y mostrado:

P1 = Puntos_I(1,:);
P2 = Puntos_I(2,:);
P3 = Puntos_I(3,:);

v1 = (P1-P2)./norm(P1-P2);
v2 = (P3-P2)./norm(P3-P2);
v_n = cross(v2,v1)./norm(cross(v2,v1));

Aeq = v_n;
beq = dot(P2,v_n);

% Parametrización del problema en el plano

A = [70.77 -40 226];
B = [70.77 40 226];
C = [70.77 -40 335];


AB = B-A;
AC = C-A;
AQ = cross(AB,AC);

T = [AB' AC' AQ' eye(3)];
T = rref(T);
TB = T(:,4:end); % Matriz de transformación de una base a la otra




Puntos_I_A = Puntos_I*0;
x0 = zeros(1,Npuntos*2);
j = 1;
for i=1:Npuntos
   Puntos_I_A(i,:) = Puntos_I(i,:)-A;
   x_newbase = TB*Puntos_I_A(i,:)';
   x0(j) = x_newbase(1);
   x0(j+1) =  x_newbase(2);
   j = j+2;
end

%scatter3(Puntos_I_A(:,1),Puntos_I_A(:,2),Puntos_I_A(:,3),'ob','LineWidth',3);

% Definición de las restricciones
lb = [0 0 0 0 0 0 0 0];
ub = [1 1 1 1 1 1 1 1];

% Minimización y búsqueda

options = optimset('Algorithm','active-set','PlotFcns',@optimplotfval,'TolFun',1e-50,'MaxFunEvals',10e5,'MaxIter',10e3);
%options = optimset('Algorithm','active-set','TolFun',1e-50,'MaxFunEvals',10e5,'MaxIter',10e3);


[x_sol,fval] = fminsearchbnd(@tre_coste_regularizado_aplicador,x0,lb,ub,options);


%

figure
scatter3(Puntos_I(:,1),Puntos_I(:,2),Puntos_I(:,3),'o','LineWidth',3);
hold on
scatter3(Origen(:,1),Origen(:,2),Origen(:,3),'ok','LineWidth',3)


X_cart = zeros(Npuntos,3);
j = 1;
for i=1:Npuntos
   X_cart(i,:) = x_sol(j)*AB + x_sol(j+1)*AC + A;
   j = j+2;
end
scatter3(X_cart(:,1),X_cart(:,2),X_cart(:,3),'om','LineWidth',3);

%

% M = [A', B', C'];
% scatter3(M(1,:),M(2,:),M(3,:),'og','LineWidth',3);

[Y,Z] = meshgrid(linspace(-40,40,50),linspace(226,335,50));
X = (beq - v_n(2)*Y + v_n(3)*Z)/v_n(1);
surf(X,Y,Z,0.5*ones(size(Z)),'EdgeAlpha',0.1,'FaceAlpha',0.5);
axis equal
hold on

save Aplicador_Bernardo

