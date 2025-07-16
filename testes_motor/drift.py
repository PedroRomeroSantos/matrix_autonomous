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
    
def frente(tempo, velocidade):
	pi.write(IN1, 1)
	pi.write(IN2, 0)
	pi.write(IN3, 1)
	pi.write(IN4, 0)
	set_pwm(ENA, velocidade)
	set_pwm(ENB, velocidade)
	time.sleep(tempo)
	parar()
	
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
# Virar direita
# =============================

def virar_direita(tempo, velocidade):
	pi.write(IN1, 1)
	pi.write(IN2, 0)
	set_pwm(ENA, velocidade)
	set_pwm(ENB, 0)
	time.sleep(tempo)
	parar()
   
   
# =============================
# Virar esquerda
# =============================

def virar_esquerda(tempo, velocidade):
	pi.write(IN3, 1)
	pi.write(IN4, 0)
	set_pwm(ENB, velocidade)
	set_pwm(ENA, 0)
	time.sleep(tempo)
	parar()

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
try:
	frente(4, velocidade=100)
	girar_sobre_eixo_pwm(4, velocidade=60, sentido='horario')
	girar_sobre_eixo_pwm(4, velocidade=60, sentido='anti-horario')
	virar_direita(4,velocidade=50)
	virar_esquerda(4,velocidade=50)
	parar()

finally:
    parar()
    pi.stop()
