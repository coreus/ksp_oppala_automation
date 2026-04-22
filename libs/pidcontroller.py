class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measured_value, dt):
        # Calculer l'erreur
        error = setpoint - measured_value
        
        # Calculer l'intégrale et la dérivée
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        # Calculer la sortie PID
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Mémoriser l'erreur pour la prochaine itération
        self.prev_error = error

        return output