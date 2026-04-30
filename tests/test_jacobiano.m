%% TEST_JACOBIANO  Verificación del Jacobiano numérico
%
%   Compara mi_jacobiano.m contra jacob0() de Peter Corke.
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
    pi/6, pi/4, pi/3, pi/6, pi/4, pi/3;
    pi/2, pi/4, 0,    pi/4, pi/2, 0;
];

tol_test = 1e-6;
todos_ok = true;

for i = 1:size(casos,1)
    q  = casos(i,:);

    J_mio  = mi_jacobiano(q);
    J_mio_pos = J_mio(1:3,:);

    J_ck   = robot.jacob0(q);
    J_ck_pos = J_ck(1:3,:);

    err    = norm(J_mio_pos - J_ck_pos, 'fro');
    ok     = err < tol_test;
    todos_ok = todos_ok && ok;

    fprintf('Caso %d — Error Frobenius VL: %.2e  %s\n', i, err, ...
        string({' ✘ FALLO',' ✔ OK'}(ok+1)));
end

if todos_ok
    fprintf('\n✔ Jacobiano verificado correctamente.\n');
else
    fprintf('\n✘ Hay discrepancias en el Jacobiano.\n');
end
