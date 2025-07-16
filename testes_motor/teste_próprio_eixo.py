import pigpio
import time

# Inicializa o pigpio
pi = pigpio.pi()

# =============================
# Definir os pinos
# =============================
IN1, IN2 = 17, 27  # Motor esquerdo
IN3, IN4 = 22, 23  # Motor direito
ENA = 19           # PWM esquerdo
ENB = 13           # PWM direito

# Configurar pinos como saída
for p in [IN1, IN2, IN3, IN4]:
    pi.set_mode(p, pigpio.OUTPUT)

# =============================
# Função PWM
# =============================
def set_pwm(pino, duty, freq=100):
    """
    duty: de 0 a 100 (%)
    """
    # Converte duty percentual para duty absoluto no range 0~255
    duty_cycle = int((duty / 100) * 255)

    pi.set_PWM_frequency(pino, freq)
    pi.set_PWM_dutycycle(pino, duty_cycle)

# =============================
# Função parar
# =============================
def parar():
    set_pwm(ENA, 0)
    set_pwm(ENB, 0)

    pi.write(IN1, 0)
    pi.write(IN2, 0)
    pi.write(IN3, 0)
    pi.write(IN4, 0)

# =============================
# Função para girar sobre o próprio eixo
# =============================
def girar_sobre_eixo_pwm(tempo, velocidade, sentido):
    if sentido == 'horario':
        pi.write(IN1, 1)
        pi.write(IN2, 0)
        pi.write(IN3, 0)
        pi.write(IN4, 1)
    elif sentido == 'anti-horario':
        pi.write(IN1, 0)
        pi.write(IN2, 1)
        pi.write(IN3, 1)
        pi.write(IN4, 0)

    set_pwm(ENA, velocidade)
    set_pwm(ENB, velocidade)

    time.sleep(tempo)
    parar()

# =============================
# Teste
# =============================
try:
    girar_sobre_eixo_pwm(10, velocidade=80, sentido='horario')
    parar()

finally:
    parar()
    pi.stop()
