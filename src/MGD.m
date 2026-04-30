function pos = MGD(q1, q2, q3, q4, q5, q6)
% MGD  Modelo Geométrico Directo — Robot 6 GDL
%
%   pos = MGD(q1, q2, q3, q4, q5, q6)
%
%   Entradas:
%       q1..q6  Ángulos de cada junta [rad]
%
%   Salida:
%       pos     Vector 4x1 homogéneo [px; py; pz; 1]
%               Posición del efector final en el frame base
%
%   Parámetros DH (UR-like, 6 GDL):
%       d1 = 0.1519  m   a2 = 0.243665 m   a3 = 0.21325 m
%       d4 = 0.11235 m   d5 = 0.08535  m   d6 = 0.0819  m
%
%   Referencia: T06 = T01*T12*T23*T34*T45*T56

    % ── Parámetros DH ────────────────────────────────────────────────────
    d1 = 0.1519;
    a2 = 0.243665;
    a3 = 0.21325;
    d4 = 0.11235;
    d5 = 0.08535;
    d6 = 0.0819;

    % ── Ángulos compuestos ───────────────────────────────────────────────
    q234 = q2 + q3 + q4;
    q23  = q2 + q3;

    % ── Columna 4 de T06 (posición del efector) ──────────────────────────
    pos = [
        d4*sin(q1) - cos(q1)*(a3*cos(q23) + a2*cos(q2)) ...
            - d6*(cos(q5)*sin(q1) - cos(q234)*cos(q1)*sin(q5)) ...
            + d5*sin(q234)*cos(q1);

        -d6*(cos(q1)*cos(q5) + cos(q234)*sin(q1)*sin(q5)) ...
            - d4*cos(q1) - sin(q1)*(a3*cos(q23) + a2*cos(q2)) ...
            + d5*sin(q234)*sin(q1);

        d1 - a3*sin(q23) - a2*sin(q2) ...
            - d5*cos(q234) + d6*sin(q234)*sin(q5);

        1
    ];
end
