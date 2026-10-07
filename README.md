# taller2
# Taller Segundo Corte — ESP32 + PyBullet

## 1. Descripción

Este proyecto integra una **ESP32 como consola física de mando** con simulaciones robóticas desarrolladas en **PyBullet**. El objetivo es demostrar el control en tiempo real de:

- **Parte A:** desplazamiento de drones entre posiciones mediante comandos enviados desde la ESP32.
- **Parte B:** posicionamiento del brazo Baxter mediante cinemática inversa y ejecución de una operación de agarre y traslado de un objeto.
- **Parte C:** consola de mando para un movimiento fluido del Baxter, con interpolación de objetivos y control de la pinza.

> **Nota:** la entrega no incluye video, de acuerdo con la indicación solicitada. La evidencia se realiza mediante capturas de pantalla de la simulación, código y explicación técnica.

## 2. Repositorios de referencia

### Parte A
[UTIAS gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones)

### Partes B y C
[erwincoumans/pybullet_robots](https://github.com/erwincoumans/pybullet_robots)

Ejemplo de Baxter utilizado como base conceptual:
[baxter_ik_demo.py](https://github.com/erwincoumans/pybullet_robots/blob/master/baxter_ik_demo.py)

Los repositorios originales se utilizan como referencia y dependencia de simulación; el código de integración ESP32/TCP es propio de este proyecto.

---

## 3. Arquitectura general

```text
                 ┌──────────────────────────────┐
                 │            ESP32             │
                 │  Joysticks + potenciometro   │
                 │  botones + Wi-Fi             │
                 └──────────────┬───────────────┘
                                │
                         TCP/IP - Wi-Fi
                                │
                                ▼
                 ┌──────────────────────────────┐
                 │       PC / Python            │
                 │  Servidor de comandos TCP    │
                 └──────────────┬───────────────┘
                                │
                   ┌────────────┴────────────┐
                   ▼                         ▼
             PyBullet / A                 PyBullet /
             gym-pybullet-drones           Baxter
                   │                         │
                   ▼                         ▼
                DRONE                 IK + motores + pinza
```

### Flujo de información

1. La ESP32 lee los controles analógicos y digitales.
2. Los valores se normalizan.
3. La ESP32 transmite una línea de texto mediante TCP.
4. Python recibe y valida el comando.
5. El servidor convierte el comando a una acción de simulación.
6. PyBullet actualiza el estado del robot.
7. El usuario observa la respuesta en tiempo real.

La ESP32 puede trabajar como cliente Wi-Fi y conectarse al PC que ejecuta el servidor TCP. El API de Arduino-ESP32 soporta Wi-Fi en modo estación y servidores TCP; Python dispone de sockets TCP para implementar el servidor. 

---

# 4. PARTE A — Movimiento de drones

## 4.1 Objetivo

Mover un dron desde una posición inicial hasta una posición final utilizando la ESP32 como consola de mando. La simulación utiliza `VelocityAviary`, que permite trabajar con entradas de velocidad y control PID interno.

La implementación oficial de `VelocityAviary` utiliza una acción de cuatro componentes por dron: velocidad relativa en X, Y, Z y una magnitud de velocidad normalizada. El controlador PID transforma esta referencia de velocidad en comandos de motor. 

## 4.2 Comandos

```text
A,x,y,z
```

donde:

- `x`: velocidad normalizada en X, entre -1 y 1.
- `y`: velocidad normalizada en Y, entre -1 y 1.
- `z`: velocidad normalizada en Z, entre -1 y 1.
- La magnitud se calcula automáticamente.

Ejemplo:

```text
A,0.50,0.00,0.20
```

significa desplazarse hacia X y subir.

El comando:

```text
STOP
```

detiene el movimiento solicitado.

## 4.3 Ejecución

### Instalar dependencias

```bash
python -m pip install -r requirements.txt
```

Para instalar la biblioteca de drones:

```bash
git clone https://github.com/utiasDSL/gym-pybullet-drones.git
cd gym-pybullet-drones
python -m pip install -e .
cd ..
```

La documentación actual del proyecto indica la instalación editable con `pip install -e .` y proporciona ejemplos de control PID y de control por velocidad. 

### Ejecutar

```bash
python parte_a/drone_server.py
```

Después de iniciar el servidor, encender la ESP32 y conectar ambos dispositivos a la misma red Wi-Fi.

## 4.4 Resultado esperado

Al mover el joystick:

- izquierda/derecha → movimiento en X;
- adelante/atrás → movimiento en Y;
- potenciómetro → movimiento vertical Z;
- botón STOP → velocidad cero.

La simulación debe mostrar el dron respondiendo a los comandos recibidos.

---

# 5. PARTE B — Baxter: posicionamiento y agarre

## 5.1 Objetivo

Desarrollar una consola que permita:

1. Controlar la posición del extremo del brazo Baxter.
2. Utilizar cinemática inversa.
3. Abrir y cerrar la pinza.
4. Acercarse a un objeto.
5. Agarrarlo.
6. Transportarlo.
7. Soltarlo en otra posición.

El repositorio `pybullet_robots` incluye ejemplos de Baxter y el archivo `baxter_ik_demo.py`. Ese ejemplo configura Baxter, define un efector final, calcula la cinemática inversa con `calculateInverseKinematics` y utiliza control de posición para los grados de libertad. 

## 5.2 Estrategia de agarre

Para hacer reproducible la práctica en PyBullet, el proyecto utiliza una **restricción fija temporal** entre el efector final y el objeto cuando se recibe `GRAB` y el objeto se encuentra suficientemente cerca.

Esto representa computacionalmente la acción de cerrar la pinza y sujetar el objeto.

Cuando se recibe:

```text
RELEASE
```

se elimina la restricción y el objeto vuelve a quedar libre.

## 5.3 Comandos

```text
B,x,y,z
GRAB
RELEASE
RESET
```

Ejemplo:

```text
B,0.20,0.10,-0.10
```

modifica la posición objetivo del efector.

## 5.4 Ejecución

Primero instalar:

```bash
python -m pip install -r requirements.txt
```

Clonar el repositorio de Baxter:

```bash
git clone https://github.com/erwincoumans/pybullet_robots.git
```

Ejecutar:

```bash
python parte_b/baxter_server.py --robot_repo ./pybullet_robots
```

### Secuencia de prueba

1. Ejecutar el servidor.
2. Verificar que Baxter aparezca.
3. Llevar el efector hacia el objeto.
4. Enviar `GRAB`.
5. Alejar el efector.
6. Comprobar que el objeto se desplaza junto con la pinza.
7. Enviar `RELEASE`.
8. Verificar que el objeto quede en la nueva posición.

---

# 6. PARTE C — Consola de mando y movimiento fluido del Baxter

## 6.1 Objetivo

Desarrollar una consola de mando con ESP32 que permita un movimiento continuo y fluido del Baxter.

A diferencia de mover el efector directamente de una coordenada a otra, se implementa una **interpolación temporal**:

```text
posición actual → posición objetivo
```

con un factor de suavizado.

Esto evita cambios bruscos de referencia y produce una respuesta más continua.

## 6.2 Control

La ESP32 transmite:

```text
C,x,y,z
```

El servidor actualiza la referencia del efector.

La posición realmente aplicada se calcula mediante:

```text
p_aplicada = p_aplicada + α(p_objetivo - p_aplicada)
```

con:

```text
0 < α < 1
```

Un valor pequeño produce un movimiento más suave.

## 6.3 Pinza

```text
GRAB
RELEASE
```

## 6.4 Reinicio

```text
RESET
```

restablece la escena y devuelve el sistema a una posición inicial.

---

# 7. Comunicación ESP32 ↔ PC

## 7.1 Protocolo

Se utiliza TCP sobre Wi-Fi.

| Comando | Función |
|---|---|
| `A,x,y,z` | Control del dron |
| `B,x,y,z` | Objetivo Baxter |
| `C,x,y,z` | Movimiento fluido Baxter |
| `GRAB` | Agarrar objeto |
| `RELEASE` | Soltar objeto |
| `STOP` | Detener movimiento |
| `RESET` | Reiniciar simulación |

## 7.2 Ventaja del protocolo

El protocolo es sencillo, legible y fácil de depurar desde el monitor serial de Arduino.

Cada comando termina con `\n`, por ejemplo:

```text
C,0.25,0.05,-0.10\n
```

---

# 8. Hardware de la consola ESP32

## Componentes

- ESP32 DevKit.
- 2 joysticks analógicos.
- 1 potenciómetro de 10 kΩ, opcional.
- 3 pulsadores.
- Resistencias de 10 kΩ si se desea usar configuración externa.
- Cable USB para programación.

## Asignación propuesta

| Elemento | GPIO |
|---|---:|
| Joystick X | 34 |
| Joystick Y | 35 |
| Potenciómetro Z | 32 |
| Botón GRAB | 25 |
| Botón RELEASE | 26 |
| Botón STOP | 27 |

Los GPIO 34 y 35 se utilizan únicamente como entradas analógicas.

---

# 9. Código de la ESP32

El archivo `esp32/esp32_console.ino` contiene el firmware.

Antes de cargarlo:

1. Cambiar `SSID`.
2. Cambiar `PASSWORD`.
3. Cambiar `SERVER_IP` por la IP del computador.
4. Mantener el puerto `5000`.
5. Seleccionar la placa ESP32 en Arduino IDE.
6. Compilar y cargar.

La ESP32 funciona como cliente TCP. El PC funciona como servidor.

---

# 10. Seguridad y validación

El servidor no ejecuta comandos arbitrarios recibidos por red. Únicamente acepta comandos definidos por el protocolo.

Además:

- X, Y y Z se limitan al rango permitido.
- Los comandos desconocidos se rechazan.
- `STOP` coloca las referencias de movimiento en cero.
- `RESET` permite recuperar el estado inicial.
- La comunicación se limita a la red local del laboratorio.

---

# 11. Pruebas

## Prueba 1 — Comunicación

Enviar:

```text
STOP
```

Resultado esperado:

```text
OK STOP
```

## Prueba 2 — Dron

Enviar:

```text
A,0.5,0,0
```

Resultado: desplazamiento positivo en X.

Enviar:

```text
A,0,0,0
```

Resultado: detención.

## Prueba 3 — Baxter

Enviar:

```text
B,0.20,0.10,-0.10
```

Resultado: cambio de la posición objetivo del efector.

## Prueba 4 — Agarre

Enviar:

```text
GRAB
```

Resultado: si el objeto está dentro de la distancia definida, queda unido al efector.

## Prueba 5 — Traslado

Cambiar la posición objetivo y comprobar que el objeto se mueve con Baxter.

## Prueba 6 — Liberación

Enviar:

```text
RELEASE
```

Resultado: el objeto queda libre.

## Prueba 7 — Suavizado

Enviar varias posiciones consecutivas mediante `C`.

Resultado: el movimiento del efector debe observarse progresivo y sin saltos de referencia.

---

# 12. Evidencias para GitHub

Como la entrega se realizará sin video, se recomienda incluir capturas de:

1. Instalación de PyBullet.
2. Consola de Python ejecutando el servidor.
3. ESP32 conectada a Wi-Fi.
4. Dron en posición inicial.
5. Dron después del desplazamiento.
6. Baxter antes del agarre.
7. Baxter con el objeto agarrado.
8. Baxter después del traslado.
9. Baxter después de liberar el objeto.
10. Código fuente de cada parte.

Guardar las imágenes en:

```text
evidencias/
```

y referenciarlas en este README mediante:

```markdown
![Evidencia 1](evidencias/evidencia_01.png)
```

---

# 13. Estructura final del repositorio

```text
Taller Segundo Corte/
│
├── README.md
├── requirements.txt
├── .gitignore
│
├── parte_a/
│   └── drone_server.py
│
├── parte_b/
│   └── baxter_server.py
│
├── parte_c/
│   └── baxter_console.py
│
├── esp32/
│   └── esp32_console.ino
│
└── evidencias/
    ├── evidencia_01.png
    ├── evidencia_02.png
    ├── evidencia_03.png
    └── ...
```

---

# 14. Análisis técnico

La arquitectura propuesta separa la **interfaz de usuario**, la **comunicación** y la **simulación**. La ESP32 no realiza la física ni la cinemática inversa; únicamente genera referencias de movimiento. Python recibe dichas referencias y las transforma en acciones sobre el simulador.

Para los drones se aprovecha el control de velocidad de `VelocityAviary`, cuyo controlador PID interno transforma la velocidad objetivo en RPM de los motores. Para Baxter se utiliza cinemática inversa para convertir una posición cartesiana del efector en posiciones articulares.

La separación es importante porque permite sustituir posteriormente la simulación por hardware real: la misma interfaz de comandos puede mantenerse y cambiar únicamente la capa de ejecución.

---

# 15. Conclusiones

1. Se diseñó una arquitectura de control distribuida entre una ESP32 y un computador.
2. La comunicación TCP permite transmitir referencias de movimiento en tiempo real sobre Wi-Fi.
3. La Parte A permite controlar el desplazamiento de un dron mediante referencias de velocidad.
4. La Parte B integra cinemática inversa y una operación de agarre y traslado de un objeto.
5. La Parte C incorpora interpolación de objetivos para obtener un movimiento más fluido del Baxter.
6. La solución queda organizada en módulos independientes y reproducibles, facilitando futuras modificaciones o una migración hacia hardware físico.

---

# 16. Referencias

- UTIAS Learning Systems and Robotics Lab, `gym-pybullet-drones`, GitHub.
- E. Coumans, `pybullet_robots`, GitHub.
- E. Coumans and Y. Bai, *PyBullet Quickstart Guide*.
- J. Panerati et al., “Learning to Fly—A Gym Environment with PyBullet Physics for Reinforcement Learning of Multi-agent Quadcopter Control,” IROS, 2021.
- Espressif Systems, Arduino-ESP32 Wi-Fi and Network API.
- Python Software Foundation, Python `socket` documentation.
