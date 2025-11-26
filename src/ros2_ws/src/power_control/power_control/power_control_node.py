import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

# GPIO numbers (BCM)
GPIO_OUT = 16   # Pin set to HIGH on startup
GPIO_IN = 22    # Pin monitored for shutdown
GPIO_PULSE = 23 # Pin for pulse signal
GPIO_RED = 26   # Red LED
GPIO_GREEN = 27 # Green LED


class PowerControlNode(Node):
    """
    ROS2 port of the power_control node:
    - drives LEDs via messages on led_control
    - toggles a pulse pin
    - watches a shutdown pin (falling edge) to call system shutdown
    """

    def __init__(self):
        super().__init__("gpio_control_node")
        self.led_state = "off"
        self.blinking = False
        self.pulse = False
        self._gpio_ok = False
        self._lock = threading.Lock()

        self.create_subscription(String, "led_control", self.led_callback, 10)

        # Pulse timer 1Hz
        self.create_timer(1.0, self.pulse_tick)
        # Blink timer 1Hz
        self.create_timer(1.0, self.blink_tick)

        self._setup_gpio()
        self.get_logger().info("power_control node ready")

    def _setup_gpio(self):
        if GPIO is None:
            self.get_logger().warn("RPi.GPIO not available; running in simulation mode")
            return
        try:
            GPIO.setwarnings(False)
            GPIO.cleanup()
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(GPIO_OUT, GPIO.OUT)
            GPIO.output(GPIO_OUT, GPIO.HIGH)

            GPIO.setup(GPIO_IN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(GPIO_IN, GPIO.FALLING, callback=self._shutdown_callback, bouncetime=300)

            GPIO.setup(GPIO_PULSE, GPIO.OUT)
            GPIO.setup(GPIO_RED, GPIO.OUT)
            GPIO.setup(GPIO_GREEN, GPIO.OUT)
            GPIO.output(GPIO_RED, GPIO.LOW)
            GPIO.output(GPIO_GREEN, GPIO.LOW)
            GPIO.setwarnings(True)
            self._gpio_ok = True
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"GPIO init failed: {exc}")
            self._gpio_ok = False

    def _shutdown_callback(self, channel):
        self.get_logger().warn(f"GPIO {GPIO_IN} triggered, shutting down")
        try:
            import subprocess
            subprocess.call(["shutdown", "now"])
        except Exception as exc:
            self.get_logger().error(f"Failed to shutdown: {exc}")

    def led_callback(self, msg: String):
        with self._lock:
            self.led_state = msg.data
            if msg.data in ("recording", "nofix"):
                self.blinking = True
            else:
                self.blinking = False
            self._apply_leds()

    def pulse_tick(self):
        if not self._gpio_ok:
            return
        self.pulse = not self.pulse
        GPIO.output(GPIO_PULSE, self.pulse)

    def blink_tick(self):
        if not self._gpio_ok:
            return
        if not self.blinking:
            return
        if self.led_state == "recording":
            GPIO.output(GPIO_RED, GPIO.LOW)
            GPIO.output(GPIO_GREEN, not GPIO.input(GPIO_GREEN))
        elif self.led_state == "nofix":
            GPIO.output(GPIO_RED, not GPIO.input(GPIO_RED))
            GPIO.output(GPIO_GREEN, not GPIO.input(GPIO_GREEN))

    def _apply_leds(self):
        if not self._gpio_ok:
            return
        state = self.led_state
        if state == "off":
            GPIO.output(GPIO_RED, GPIO.LOW)
            GPIO.output(GPIO_GREEN, GPIO.LOW)
        elif state == "ready":
            GPIO.output(GPIO_RED, GPIO.LOW)
            GPIO.output(GPIO_GREEN, GPIO.HIGH)
        elif state == "recording":
            GPIO.output(GPIO_RED, GPIO.LOW)
            GPIO.output(GPIO_GREEN, GPIO.HIGH)
        elif state == "warning":
            GPIO.output(GPIO_RED, GPIO.HIGH)
            GPIO.output(GPIO_GREEN, GPIO.HIGH)
        elif state == "error":
            GPIO.output(GPIO_RED, GPIO.HIGH)
            GPIO.output(GPIO_GREEN, GPIO.LOW)
        elif state == "nofix":
            GPIO.output(GPIO_RED, GPIO.HIGH)
            GPIO.output(GPIO_GREEN, GPIO.HIGH)

    def destroy_node(self):
        if self._gpio_ok:
            GPIO.output(GPIO_OUT, GPIO.LOW)
            GPIO.output(GPIO_RED, GPIO.LOW)
            GPIO.output(GPIO_GREEN, GPIO.LOW)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PowerControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
