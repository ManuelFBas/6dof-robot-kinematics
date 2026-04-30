%% TEST_MGD  Verificación del Modelo Geométrico Directo
%
%   Compara MGD.m contra fkine() de Peter Corke para
%   distintas configuraciones de prueba.
clear; clc;

%% ── Robot Corke (referencia) ─────────────────────────────────────────────
d1=0.1519; a2=0.243665; a3=0.21325;
d4=0.11235; d5=0.08535; d6=0.0819;

L(1) = Link('revolute','d',d1,'a',0,  'alpha', pi/2);
L(2) = Link('revolute','d',0, 'a',-a2,'alpha', 0   );
L(3) = Link('revolute','d',0, 'a',-a3,'alpha', 0   );
L(4) = Link('revolute','d',d4,'a',0,  'alpha', pi/2);
L(5) = Link('revolute','d',d5,'a',0,  'alpha',-pi/2);
L(6) = Link('revolute','d',d6,'a',0,  'alpha', 0   );
robot = SerialLink(L);

%% ── Casos de prueba ──────────────────────────────────────────────────────
casos = [
    0,      0,      0,      0,      0,      0;
    pi/6,   pi/4,   pi/3,   pi/6,   pi/4,   pi/3;
    pi/2,   pi/4,   0,      pi/4,   pi/2,   0;
    -pi/4, -pi/3,   pi/6,   pi/4,  -pi/6,   pi/3;
    pi,     pi/2,  -pi/4,   0,      pi/3,  -pi/2;
];

fprintf('%-6s  %-30s  %-30s  %-10s\n','Caso','p_MGD (m)','p_Corke (m)','Error');
fprintf('%s\n', repmat('-',1,85));

tol_test = 1e-8;
todos_ok = true;

for i = 1:size(casos,1)
    q = casos(i,:);

    % Tu MGD
    p_mgd  = MGD(q(1),q(2),q(3),q(4),q(5),q(6));
    p_mgd  = p_mgd(1:3);

    % Corke (referencia)
    T_ck   = robot.fkine(q);
    p_ck   = T_ck.t;

    err    = norm(p_mgd - p_ck);
    ok     = err < tol_test;
    todos_ok = todos_ok && ok;
    estado = '✔ OK';
    if ~ok; estado = '✘ FALLO'; end

    fprintf('%-6d  [%7.4f %7.4f %7.4f]  [%7.4f %7.4f %7.4f]  %.2e  %s\n', ...
        i, p_mgd', p_ck', err, estado);
end

fprintf('%s\n', repmat('-',1,85));
if todos_ok
    fprintf('✔ Todos los casos superados  (tol = %.0e)\n', tol_test);
else
    fprintf('✘ Hay casos con error superior a la tolerancia.\n');
end
