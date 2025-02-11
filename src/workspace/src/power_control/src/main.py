#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import subprocess
import time
from std_msgs.msg import String  # To receive LED control commands

# GPIO numbers (BCM mode)
GPIO_OUT = 16   # Pin set to HIGH on startup
GPIO_IN  = 22   # Pin monitored for shutdown
GPIO_Pulse = 23 # Pin for pulse signal

GPIO_RED = 26   # Red LED
GPIO_GREEN = 27 # Green LED



def shutdown_callback():
    """ Callback triggered when GPIO_IN detects a rising edge, shutting down the Raspberry Pi. """
    rospy.logwarn("GPIO {} detected: Initiating shutdown...".format(GPIO_IN))
    subprocess.call(["shutdown", "now"])

def on_shutdown():
    """ Function executed when ROS shuts down the node. It turns off all LEDs before exiting. """
    rospy.loginfo("Shutdown: Turning off all LEDs.")
    GPIO.output(GPIO_OUT, GPIO.LOW)
    GPIO.output(GPIO_RED, GPIO.LOW)
    GPIO.output(GPIO_GREEN, GPIO.LOW)
    GPIO.cleanup()

def set_led_state(state):
    """ Turns LEDs on/off based on the given state """
    global blinking, led_state
    led_state = state
    blinking = False

    if state == "off":
        GPIO.output(GPIO_RED, GPIO.LOW)
        GPIO.output(GPIO_GREEN, GPIO.LOW)
    elif state == "ready":
        GPIO.output(GPIO_RED, GPIO.LOW)
        GPIO.output(GPIO_GREEN, GPIO.HIGH)
    elif state == "recording":
        blinking = True  # Green LED blinking at 1Hz
    elif state == "warning":
        GPIO.output(GPIO_RED, GPIO.HIGH)
        GPIO.output(GPIO_GREEN, GPIO.HIGH)
    elif state == "error":
        GPIO.output(GPIO_RED, GPIO.HIGH)
        GPIO.output(GPIO_GREEN, GPIO.LOW)
    elif state == "nofix":
        blinking = True  # Both LEDs blinking at 1Hz

def led_control_callback(msg):
    """ ROS callback to update LED state based on received message """
    rospy.loginfo(f"Received LED state: {msg.data}")
    set_led_state(msg.data)

def main():
    """ Main ROS loop """
    rospy.init_node('gpio_control_node', anonymous=True)
    rospy.on_shutdown(on_shutdown)

    # Subscribe to the topic to receive LED state commands
    rospy.Subscriber("led_control", String, led_control_callback)

    # Initialize GPIO
    GPIO.setmode(GPIO.BCM)

    # Main output
    GPIO.setup(GPIO_OUT, GPIO.OUT)
    GPIO.output(GPIO_OUT, GPIO.HIGH)

    # Input for shutdown button
    GPIO.setup(GPIO_IN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(GPIO_IN, GPIO.FALLING, callback=lambda _: shutdown_callback(), bouncetime=300)

    # Pulse LED
    GPIO.setup(GPIO_Pulse, GPIO.OUT)

    # Red and Green LEDs
    GPIO.setup(GPIO_RED, GPIO.OUT)
    GPIO.setup(GPIO_GREEN, GPIO.OUT)
    GPIO.output(GPIO_RED, GPIO.LOW)
    GPIO.output(GPIO_GREEN, GPIO.LOW)

    # Blinking state management
    blinking = False
    led_state = "off"

    rate = rospy.Rate(1)  # 1Hz frequency for blinking LEDs
    pulse = False

    while not rospy.is_shutdown():
        pulse = not pulse
        GPIO.output(GPIO_Pulse, pulse)

        # Handle blinking LED states
        if blinking:
            if led_state == "recording":
                GPIO.output(GPIO_RED, GPIO.LOW)
                GPIO.output(GPIO_GREEN, not GPIO.input(GPIO_GREEN))  # Green blinking
            elif led_state == "nofix":
                GPIO.output(GPIO_RED, not GPIO.input(GPIO_RED))
