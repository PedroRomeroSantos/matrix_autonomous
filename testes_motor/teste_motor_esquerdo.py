import pigpio
import time

pi = pigpio.pi()

IN1 = 17
IN2 = 27
ENA = 19

pi.set_mode(IN1, pigpio.OUTPUT)
pi.set_mode(IN2, pigpio.OUTPUT)

# Seta PWM em alto
pi.set_PWM_dutycycle(ENA, 255)

# Motor pra frente
pi.write(IN1, 1)
pi.write(IN2, 0)
time.sleep(3)

# Motor pra trás
pi.write(IN1, 0)
pi.write(IN2, 1)
time.sleep(3)

# Parar
pi.set_PWM_dutycycle(ENA, 0)
pi.write(IN1, 0)
pi.write(IN2, 0)

pi.stop()
