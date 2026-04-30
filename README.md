# Robot 6 GDL — Cinemática Directa e Inversa

Implementación completa del **Modelo Geométrico Directo (MGD)**, **Jacobiano geométrico** y **Cinemática Inversa** mediante el método de Newton-Raphson para un robot serial de 6 grados de libertad.

---

## Contenido

```
robot_6gdl/
├── src/
│   ├── MGD.m                  # Modelo Geométrico Directo (columna 4 de T06)
│   ├── mi_jacobiano.m         # Jacobiano geométrico 6×6 [VL; VA]
│   ├── cinematica_inversa.m   # Cinemática inversa — Newton-Raphson  ← MAIN
│   ├── MGD_simbolico.m        # Derivación simbólica de T06 (Symbolic Toolbox)
│   └── jacobiano_simbolico.m  # Derivación simbólica del Jacobiano
├── tests/
│   ├── test_MGD.m             # Verifica MGD contra Peter Corke
│   └── test_jacobiano.m       # Verifica Jacobiano contra Peter Corke
├── docs/
│   └── DH_parametros.md       # Tabla de parámetros Denavit-Hartenberg
├── results/                   # Gráficas generadas (ignoradas por git)
└── README.md
```

---

## Parámetros Denavit-Hartenberg

| Junta | θᵢ | dᵢ (m)   | aᵢ (m)     | αᵢ      |
|-------|-----|----------|------------|---------|
| 1     | q1  | 0.1519   | 0          | π/2     |
| 2     | q2  | 0        | −0.243665  | 0       |
| 3     | q3  | 0        | −0.21325   | 0       |
| 4     | q4  | 0.11235  | 0          | π/2     |
| 5     | q5  | 0.08535  | 0          | −π/2    |
| 6     | q6  | 0.0819   | 0          | 0       |

---

## Modelo Geométrico Directo

La posición del efector final se obtiene de la columna 4 de:

$$T_0^6 = T_0^1 \cdot T_1^2 \cdot T_2^3 \cdot T_3^4 \cdot T_4^5 \cdot T_5^6$$

```matlab
pos = MGD(q1, q2, q3, q4, q5, q6);
% pos → vector 4×1 homogéneo [px; py; pz; 1]
```

Con ángulos compuestos q₂₃ = q2+q3 y q₂₃₄ = q2+q3+q4:

$$p_x = d_4\sin q_1 - \cos q_1(a_3\cos q_{23} + a_2\cos q_2) - d_6(\cos q_5 \sin q_1 - \cos q_{234}\cos q_1\sin q_5) + d_5\sin q_{234}\cos q_1$$

$$p_y = d_6(\cos q_1\cos q_5 + \cos q_{234}\sin q_1\sin q_5) - d_4\cos q_1 - \sin q_1(a_3\cos q_{23} + a_2\cos q_2) + d_5\sin q_{234}\sin q_1$$

$$p_z = d_1 - a_3\sin q_{23} - a_2\sin q_2 - d_5\cos q_{234} + d_6\sin q_{234}\sin q_5$$

---

## Jacobiano Geométrico

```matlab
J = mi_jacobiano(q);   % q = [q1 q2 q3 q4 q5 q6]  →  J: 6×6
```

Estructura del Jacobiano:

$$J = \begin{bmatrix} J_{VL} \\ J_{VA} \end{bmatrix} \quad \text{donde} \quad J_{VL} = \frac{\partial \mathbf{p}}{\partial \mathbf{q}}, \quad J_{VA} = [\mathbf{z}_0 \ \mathbf{z}_1 \ \mathbf{z}_2 \ \mathbf{z}_3 \ \mathbf{z}_4 \ \mathbf{z}_5]$$

- **VL (3×6):** derivadas parciales de [px, py, pz] respecto a cada qᵢ
- **VA (3×6):** eje Z del frame anterior a cada junta (extraído de T0i)

---

## Cinemática Inversa — Newton-Raphson

### Algoritmo

```
q ← q₀ (configuración inicial)
repetir:
    p_actual  = MGD(q)
    e         = p_objetivo − p_actual
    si ‖e‖ < tol  →  STOP ✔
    J_pos     = mi_jacobiano(q)[1:3, :]        # solo velocidad lineal
    J⁺        = Jᵀ · (J·Jᵀ + λ²·I)⁻¹          # pseudo-inversa amortiguada
    dq        = J⁺ · e
    q        ← q + dq
```

### Parámetros configurables

| Parámetro | Valor por defecto | Descripción |
|-----------|-------------------|-------------|
| `q0`      | `[0 0 0 0 0 0]`   | Configuración inicial |
| `tol`     | `1e-6`            | Tolerancia de convergencia (m) |
| `max_iter`| `200`             | Máximo de iteraciones |
| `lambda`  | `0.1`             | Amortiguamiento pseudo-inversa |

### Ejecución

```matlab
% Abrir MATLAB y situarse en la carpeta src/
cd src
cinematica_inversa
```

---

## Requisitos

| Herramienta | Versión mínima |
|-------------|---------------|
| MATLAB | R2020a |
| Peter Corke Robotics Toolbox | 10.x |
| Symbolic Math Toolbox | (solo para scripts `_simbolico.m`) |

### Instalar Peter Corke Toolbox

```matlab
% Opción 1 — desde MATLAB Add-On Explorer
% Buscar: "Robotics Toolbox for MATLAB" de Peter Corke

% Opción 2 — manual
% Descargar desde: https://petercorke.com/toolboxes/robotics-toolbox/
% Ejecutar: startup_rvc
```

---

## Tests

```matlab
cd tests

% Verificar MGD contra fkine() de Corke
test_MGD

% Verificar Jacobiano contra jacob0() de Corke
test_jacobiano
```

Salida esperada:
```
Caso 1  [px py pz]_MGD == [px py pz]_Corke  error: 1.2e-15  ✔ OK
...
✔ Todos los casos superados  (tol = 1e-08)
```

---

## Rol de Peter Corke en este proyecto

Peter Corke se usa **únicamente para visualización 3D y tests de verificación**.
La cinemática es resuelta completamente con las funciones propias.

| Función Corke | Uso en este proyecto |
|---------------|----------------------|
| `SerialLink`  | Definir el robot para `plot()` |
| `robot.plot(q)` | Visualización 3D de la configuración |
| `robot.fkine(q)` | **Solo en tests** — referencia para validar `MGD.m` |
| `robot.jacob0(q)` | **Solo en tests** — referencia para validar `mi_jacobiano.m` |

---

## Resultados típicos

```
╔══════════════════════════════════════════╗
║   CINEMÁTICA INVERSA — Newton-Raphson    ║
╚══════════════════════════════════════════╝

Posición objetivo : [0.1842  -0.0651  0.2134] m

Iter    ||error||
----------------------
1       0.18421300
2       0.04321100
...
23      0.00000087
✔ Convergido en iteración 23  |  error = 8.7e-7

q_real       : 0.524  0.785  1.047  0.524  0.785  1.047
q_calculado  : 0.524  0.785  1.047  0.524  0.785  1.047
Error final  : 8.7e-07 m
```

---

## Autores

Proyecto desarrollado para la asignatura de **Robótica** — Ingeniería en Mecatrónica.
