%% JACOBIANO_SIMBOLICO  Jacobiano geométrico simbólico — Robot 6 GDL
%
%   Calcula J = [VL; VA] de forma simbólica y lo muestra.
%   VL = jacobian(pos, Q)    → velocidad lineal
%   VA = ejes Z de T0i       → velocidad angular
%
%   Ejecutar este script UNA SOLA VEZ para verificar/regenerar mi_jacobiano.m
clear; clc;

syms q1 q2 q3 q4 q5 q6 d1 a2 a3 d4 d5 d6 real

%% ── Posición del efector (de MGD_simbolico.m) ────────────────────────────
q234 = q2+q3+q4;
q23  = q2+q3;

pos = [
    d4*sin(q1) - cos(q1)*(a3*cos(q23)+a2*cos(q2)) ...
        - d6*(cos(q5)*sin(q1)-cos(q234)*cos(q1)*sin(q5)) ...
        + d5*sin(q234)*cos(q1);
    -d6*(cos(q1)*cos(q5)+cos(q234)*sin(q1)*sin(q5)) ...
        - d4*cos(q1) - sin(q1)*(a3*cos(q23)+a2*cos(q2)) ...
        + d5*sin(q234)*sin(q1);
    d1 - a3*sin(q23) - a2*sin(q2) ...
        - d5*cos(q234) + d6*sin(q234)*sin(q5);
    1
];

%% ── VL: jacobiano simbólico de la posición ───────────────────────────────
Q  = [q1,q2,q3,q4,q5,q6];
VL = simplify(jacobian(pos(1:3), Q));

disp('=== VL — Velocidad lineal (3x6) ===');
disp(VL)

%% ── T0i acumuladas ───────────────────────────────────────────────────────
T01 = [cos(q1), 0,  sin(q1), 0;
       sin(q1), 0, -cos(q1), 0;
       0,       1,  0,       d1;
       0,       0,  0,       1];

T02 = simplify(T01*[cos(q2),-sin(q2),0,-a2*cos(q2);
                    sin(q2), cos(q2),0,-a2*sin(q2);
                    0,0,1,0; 0,0,0,1]);

T03 = simplify(T02*[cos(q3),-sin(q3),0,-a3*cos(q3);
                    sin(q3), cos(q3),0,-a3*sin(q3);
                    0,0,1,0; 0,0,0,1]);

T04 = simplify(T03*[cos(q4),0, sin(q4),0;
                    sin(q4),0,-cos(q4),0;
                    0,1,0,d4; 0,0,0,1]);

T05 = simplify(T04*[cos(q5),0, sin(q5),0;
                    sin(q5),0,-cos(q5),0;
                    0,-1,0,d5; 0,0,0,1]);

%% ── VA: ejes Z de cada frame ─────────────────────────────────────────────
z0 = [0;0;1];
z1 = T01(1:3,3);
z2 = T02(1:3,3);
z3 = T03(1:3,3);
z4 = T04(1:3,3);
z5 = T05(1:3,3);

VA = simplify([z0,z1,z2,z3,z4,z5]);

disp('=== VA — Velocidad angular (3x6) ===');
disp(VA)

%% ── Jacobiano completo ───────────────────────────────────────────────────
J = simplify([VL; VA]);

disp('=== J — Jacobiano completo (6x6) ===');
disp(J)

%% ── Determinante (singularidades) ────────────────────────────────────────
fprintf('Calculando det(J)...\n');
detJ = simplify(det(J));
disp('det(J) ='); disp(detJ)
