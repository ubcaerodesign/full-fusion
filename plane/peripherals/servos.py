import pigpio
import time
import logging


class Servos:
    """Opens and Closes servos by moving them to a hard-coded position."""

    def __init__(self):
        try:
            self.pwm = pigpio.pi()
            self.servo = 4
            self.open_pos = 1800
            self.close_pos = 1150
            self.pwm.set_mode(self.servo, pigpio.OUTPUT)
            self.pwm.set_PWM_frequency(self.servo, 50)

            # Reset servo
            self.reset()

        except Exception as e:
            print(e)
            return

    def reset(self):
        logging.info("Resetting servo")
        self.pwm.set_servo_pulsewidth(self.servo, self.open_pos)
        time.sleep(1)

        # Turning off servo
        self.pwm.set_PWM_dutycycle(self.servo, 0)
        self.pwm.set_PWM_frequency(self.servo, 0)

    def drop(self):
        logging.info("Dropping servo")
        self.pwm.set_servo_pulsewidth(self.servo, self.close_pos)
        time.sleep(1)

        # Turning off servo
        self.pwm.set_PWM_dutycycle(self.servo, 0)
        self.pwm.set_PWM_frequency(self.servo, 0)


if __name__ == "__main__":
    servos = Servos()
    servos.drop()
