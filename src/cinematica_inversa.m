%% CINEMATICA_INVERSA  Newton-Raphson con Jacobiano numérico
%
%   Calcula la configuración de juntas q = [q1..q6] que lleva el efector
%   a una posición cartesiana deseada [px, py, pz].
%
%   Funciones propias utilizadas:
%       MGD(q1..q6)      → posición del efector (columna 4 de T06)
%       mi_jacobiano(q)  → Jacobiano geométrico 6x6
%
%   Visualización:
%       Peter Corke Robotics Toolbox (SerialLink + robot.plot)
%
%   Algoritmo:
%       e    = p_target - MGD(q)
%       J⁺   = Jᵀ(JJᵀ + λ²I)⁻¹    pseudo-inversa amortiguada
%       dq   = J⁺ · e
%       q   ← q + dq
%       repetir hasta ‖e‖ < tol
%
clear; clc; close all;

%% ── 1. ROBOT con Corke (solo para visualización 3D) ─────────────────────
d1=0.1519; a2=0.243665; a3=0.21325;
d4=0.11235; d5=0.08535; d6=0.0819;

L(1) = Link('revolute','d',d1,'a',0,  'alpha', pi/2);
L(2) = Link('revolute','d',0, 'a',-a2,'alpha', 0   );
L(3) = Link('revolute','d',0, 'a',-a3,'alpha', 0   );
L(4) = Link('revolute','d',d4,'a',0,  'alpha', pi/2);
L(5) = Link('revolute','d',d5,'a',0,  'alpha',-pi/2);
L(6) = Link('revolute','d',d6,'a',0,  'alpha', 0   );

robot = SerialLink(L, 'name', 'Robot 6GDL');

%% ── 2. POSE OBJETIVO usando tu MGD ──────────────────────────────────────
q_real   = [pi/6, pi/4, pi/3, pi/6, pi/4, pi/3];

T_target = MGD(q_real(1),q_real(2),q_real(3), ...
               q_real(4),q_real(5),q_real(6));
p_target = T_target(1:3);

fprintf('╔══════════════════════════════════════════╗\n');
fprintf('║   CINEMÁTICA INVERSA — Newton-Raphson    ║\n');
fprintf('╚══════════════════════════════════════════╝\n\n');
fprintf('Posición objetivo : [%.4f  %.4f  %.4f] m\n', p_target');

%% ── 3. NEWTON-RAPHSON ────────────────────────────────────────────────────
q        = zeros(1,6);     % configuración inicial
tol      = 1e-6;           % tolerancia de convergencia
max_iter = 200;            % máximo de iteraciones
lambda   = 0.1;            % amortiguamiento (evita singularidades)
historial = zeros(max_iter,1);

fprintf('\n%-6s  %-14s\n','Iter','||error||');
fprintf('%s\n', repmat('-',1,22));

for k = 1:max_iter

    % Posición actual con tu MGD
    T_actual = MGD(q(1),q(2),q(3),q(4),q(5),q(6));
    p_actual = T_actual(1:3);

    % Error de posición
    e       = p_target - p_actual;
    norma_e = norm(e);
    historial(k) = norma_e;

    fprintf('%-6d  %-14.8f\n', k, norma_e);

    if norma_e < tol
        fprintf('\n✔ Convergido en iteración %d  |  error = %.2e\n', k, norma_e);
        break;
    end

    % Tu Jacobiano — solo filas de velocidad lineal (1:3)
    J     = mi_jacobiano(q);
    J_pos = J(1:3,:);

    % Pseudo-inversa amortiguada
    J_pinv = J_pos' / (J_pos*J_pos' + lambda^2*eye(3));

    % Actualizar configuración
    dq = (J_pinv * e)';
    q  = q + dq;
end

%% ── 4. RESULTADOS ────────────────────────────────────────────────────────
T_final = MGD(q(1),q(2),q(3),q(4),q(5),q(6));
p_final = T_final(1:3);

fprintf('\n╔══════════════════════════════════════════╗\n');
fprintf('║              RESULTADOS                  ║\n');
fprintf('╠══════════════════════════════════════════╣\n');
fprintf('║ %-12s: %s\n║\n', 'q_real',       sprintf('%.3f ',q_real));
fprintf('║ %-12s: %s\n║\n', 'q_calculado',  sprintf('%.3f ',q));
fprintf('╠══════════════════════════════════════════╣\n');
fprintf('║ Pos. objetivo  : [%.4f %.4f %.4f]\n', p_target');
fprintf('║ Pos. calculada : [%.4f %.4f %.4f]\n', p_final');
fprintf('║ Error final    : %.2e m\n', norm(p_target-p_final));
fprintf('╚══════════════════════════════════════════╝\n');

%% ── 5. VISUALIZACIÓN 3D con Corke ───────────────────────────────────────
ws = [-0.6 0.6 -0.6 0.6 0 0.6];

figure('Color','white','Name','Cinemática Inversa');
subplot(1,2,1)
  robot.plot(q_real,'workspace',ws,'noname');
  title('Configuración real (q\_real)','FontWeight','normal');

subplot(1,2,2)
  robot.plot(q,'workspace',ws,'noname');
  title('Configuración calculada (IK)','FontWeight','normal');

%% ── 6. GRÁFICA DE CONVERGENCIA ───────────────────────────────────────────
figure('Color','white','Name','Convergencia Newton-Raphson');
semilogy(1:k, historial(1:k),'b-o','LineWidth',2,'MarkerSize',5,'MarkerFaceColor','b');
hold on;
yline(tol,'r--','LineWidth',1.5,'Label','Tolerancia 1e-6');
xlabel('Iteración','FontSize',12);
ylabel('‖error‖  (escala log)','FontSize',12);
title('Convergencia del método Newton-Raphson','FontWeight','normal','FontSize',13);
grid on; grid minor;
legend('Norma del error','Tolerancia','Location','southwest');
