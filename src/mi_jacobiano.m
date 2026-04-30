function J = mi_jacobiano(q)
% MI_JACOBIANO  Jacobiano geométrico 6x6 — Robot 6 GDL
%
%   J = mi_jacobiano(q)
%
%   Entrada:
%       q   Vector 1x6 con los ángulos de junta [q1 q2 q3 q4 q5 q6] [rad]
%
%   Salida:
%       J   Matriz 6x6 — Jacobiano geométrico en el frame base
%           Filas 1:3 → velocidad lineal  (VL)
%           Filas 4:6 → velocidad angular (VA)
%
%   Método:
%       VL = jacobian(pos, Q)     derivada parcial del MGD
%       VA = ejes Z de T01..T05   columna 3 de cada T acumulada

    q1=q(1); q2=q(2); q3=q(3); q4=q(4); q5=q(5); q6=q(6); %#ok<NASGU>

    % ── Parámetros DH ────────────────────────────────────────────────────
    d1 = 0.1519;
    a2 = 0.243665;
    a3 = 0.21325;
    d4 = 0.11235;
    d5 = 0.08535;
    d6 = 0.0819;

    q234 = q2+q3+q4;
    q23  = q2+q3;

    % ── VL: derivadas parciales del MGD respecto a cada qi ───────────────
    VL = [
        d4*cos(q1)+sin(q1)*(a3*cos(q23)+a2*cos(q2)) ...
            -d6*(cos(q5)*cos(q1)+cos(q234)*sin(q1)*sin(q5)) ...
            -d5*sin(q234)*sin(q1), ...
        -sin(q1)*(a3*sin(q23)+a2*sin(q2)) ...
            +d6*cos(q1)*sin(q5)*sin(q234) ...
            +d5*cos(q1)*cos(q234), ...
        -sin(q1)*a3*sin(q23) ...
            +d6*cos(q1)*sin(q5)*sin(q234) ...
            +d5*cos(q1)*cos(q234), ...
        d6*cos(q1)*sin(q5)*sin(q234)+d5*cos(q1)*cos(q234), ...
        -d6*(sin(q5)*sin(q1)+cos(q234)*cos(q5)*cos(q1)), ...
        0;

        -d4*sin(q1)-cos(q1)*(a3*cos(q23)+a2*cos(q2)) ...
            +d6*(cos(q5)*sin(q1)-cos(q234)*cos(q1)*sin(q5)) ...
            +d5*sin(q234)*cos(q1), ...
        -cos(q1)*(a3*sin(q23)+a2*sin(q2)) ...
            -d6*sin(q1)*sin(q5)*sin(q234) ...
            -d5*sin(q1)*cos(q234), ...
        -cos(q1)*a3*sin(q23) ...
            -d6*sin(q1)*sin(q5)*sin(q234) ...
            -d5*sin(q1)*cos(q234), ...
        -d6*sin(q1)*sin(q5)*sin(q234)-d5*sin(q1)*cos(q234), ...
        -d6*(cos(q5)*cos(q1)+cos(q234)*sin(q1)*sin(q5))*(-1), ...
        0;

        0, ...
        -a3*cos(q23)-a2*cos(q2)+d6*sin(q5)*sin(q234)-d5*cos(q234), ...
        -a3*cos(q23)-d6*sin(q5)*cos(q234)*(-1)-d5*cos(q234), ...
        -d6*sin(q5)*cos(q234)*(-1)-d5*cos(q234), ...
        d6*cos(q5)*sin(q234), ...
        0
    ];

    % ── Matrices de transformación acumuladas T0i ─────────────────────────
    T01 = [cos(q1), 0,  sin(q1), 0;
           sin(q1), 0, -cos(q1), 0;
           0,       1,  0,       d1;
           0,       0,  0,       1];

    T02 = T01 * [cos(q2),-sin(q2), 0,-a2*cos(q2);
                 sin(q2), cos(q2), 0,-a2*sin(q2);
                 0,       0,       1, 0;
                 0,       0,       0, 1];

    T03 = T02 * [cos(q3),-sin(q3), 0,-a3*cos(q3);
                 sin(q3), cos(q3), 0,-a3*sin(q3);
                 0,       0,       1, 0;
                 0,       0,       0, 1];

    T04 = T03 * [cos(q4), 0, sin(q4), 0;
                 sin(q4), 0,-cos(q4), 0;
                 0,       1, 0,       d4;
                 0,       0, 0,       1];

    T05 = T04 * [cos(q5), 0, sin(q5), 0;
                 sin(q5), 0,-cos(q5), 0;
                 0,      -1, 0,       d5;
                 0,       0, 0,       1];

    % ── VA: ejes Z de cada frame (columna 3 de T0i) ───────────────────────
    VA = [[0;0;1], T01(1:3,3), T02(1:3,3), T03(1:3,3), T04(1:3,3), T05(1:3,3)];

    % ── Jacobiano completo 6×6 ────────────────────────────────────────────
    J = [VL; VA];
end
