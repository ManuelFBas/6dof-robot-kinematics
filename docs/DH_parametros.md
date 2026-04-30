# Parámetros Denavit-Hartenberg — Robot 6 GDL

## Convención DH estándar

Cada junta se describe con cuatro parámetros: `[θᵢ, dᵢ, aᵢ, αᵢ]`

La matriz de transformación entre frames consecutivos es:

$$T_{i-1}^{i} = R_z(\theta_i) \cdot T_z(d_i) \cdot T_x(a_i) \cdot R_x(\alpha_i)$$

## Tabla de parámetros

| Junta | θᵢ (variable) | dᵢ (m)     | aᵢ (m)      | αᵢ (rad) |
|-------|---------------|------------|-------------|----------|
| 1     | q1            |  0.1519    |  0          |  π/2     |
| 2     | q2            |  0         | −0.243665   |  0       |
| 3     | q3            |  0         | −0.21325    |  0       |
| 4     | q4            |  0.11235   |  0          |  π/2     |
| 5     | q5            |  0.08535   |  0          | −π/2     |
| 6     | q6            |  0.0819    |  0          |  0       |

## Matrices individuales

### T01
```
T01 = [cos(q1), 0,  sin(q1), 0   ]
      [sin(q1), 0, -cos(q1), 0   ]
      [0,       1,  0,       d1  ]
      [0,       0,  0,       1   ]
```

### T12
```
T12 = [cos(q2), -sin(q2), 0, -a2·cos(q2)]
      [sin(q2),  cos(q2), 0, -a2·sin(q2)]
      [0,        0,       1,  0          ]
      [0,        0,       0,  1          ]
```

### T23
```
T23 = [cos(q3), -sin(q3), 0, -a3·cos(q3)]
      [sin(q3),  cos(q3), 0, -a3·sin(q3)]
      [0,        0,       1,  0          ]
      [0,        0,       0,  1          ]
```

### T34
```
T34 = [cos(q4), 0,  sin(q4), 0 ]
      [sin(q4), 0, -cos(q4), 0 ]
      [0,       1,  0,       d4]
      [0,       0,  0,       1 ]
```

### T45
```
T45 = [cos(q5), 0,  sin(q5), 0 ]
      [sin(q5), 0, -cos(q5), 0 ]
      [0,      -1,  0,       d5]
      [0,       0,  0,       1 ]
```

### T56
```
T56 = [cos(q6), -sin(q6), 0, 0 ]
      [sin(q6),  cos(q6), 0, 0 ]
      [0,        0,       1, d6]
      [0,        0,       0, 1 ]
```

## MGD completo

$$T_0^6 = T_0^1 \cdot T_1^2 \cdot T_2^3 \cdot T_3^4 \cdot T_4^5 \cdot T_5^6$$
