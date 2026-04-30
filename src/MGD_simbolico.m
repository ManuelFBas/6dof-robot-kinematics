%% MGD_SIMBOLICO  Modelo Geométrico Directo simbólico — Robot 6 GDL
%
%   Calcula T06 = T01*T12*T23*T34*T45*T56 con SymPy-MATLAB (Symbolic Toolbox)
%   y extrae la columna de posición.
%
%   Ejecutar este script UNA SOLA VEZ para verificar/regenerar MGD.m
clear; clc;

syms q1 q2 q3 q4 q5 q6 d1 a2 a3 d4 d5 d6 real

%% ── Matrices DH individuales ─────────────────────────────────────────────
T01 = [cos(q1), 0,  sin(q1), 0;
       sin(q1), 0, -cos(q1), 0;
       0,       1,  0,       d1;
       0,       0,  0,       1];

T12 = [cos(q2), -sin(q2), 0, -a2*cos(q2);
       sin(q2),  cos(q2), 0, -a2*sin(q2);
       0,        0,       1,  0;
       0,        0,       0,  1];

T23 = [cos(q3), -sin(q3), 0, -a3*cos(q3);
       sin(q3),  cos(q3), 0, -a3*sin(q3);
       0,        0,       1,  0;
       0,        0,       0,  1];

T34 = [cos(q4), 0,  sin(q4), 0;
       sin(q4), 0, -cos(q4), 0;
       0,       1,  0,       d4;
       0,       0,  0,       1];

T45 = [cos(q5), 0,  sin(q5), 0;
       sin(q5), 0, -cos(q5), 0;
       0,      -1,  0,       d5;
       0,       0,  0,       1];

T56 = [cos(q6), -sin(q6), 0, 0;
       sin(q6),  cos(q6), 0, 0;
       0,        0,       1, d6;
       0,        0,       0, 1];

%% ── MGD completo ─────────────────────────────────────────────────────────
fprintf('Calculando T06 (puede tardar unos segundos)...\n');
T06 = simplify(T01*T12*T23*T34*T45*T56);

disp('=== MGD — Matriz T06 ===');
disp(T06)

%% ── Columna de posición ──────────────────────────────────────────────────
px = simplify(T06(1,4));
py = simplify(T06(2,4));
pz = simplify(T06(3,4));

fprintf('\n=== Posición del efector final ===\n');
fprintf('px = '); disp(px)
fprintf('py = '); disp(py)
fprintf('pz = '); disp(pz)

%% ── Verificación numérica en home (todos qi = 0) ─────────────────────────
vals = {q1,0, q2,0, q3,0, q4,0, q5,0, q6,0, ...
        d1,0.1519, a2,0.243665, a3,0.21325, ...
        d4,0.11235, d5,0.08535, d6,0.0819};

p_home = double(subs([px;py;pz], vals(1:2:end), vals(2:2:end)));
fprintf('\n=== Posición HOME (todos qi=0) ===\n');
fprintf('px = %.6f m\n', p_home(1));
fprintf('py = %.6f m\n', p_home(2));
fprintf('pz = %.6f m\n', p_home(3));
