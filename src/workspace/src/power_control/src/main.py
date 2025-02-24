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

# Initialize global variables for LED control
blinking = False
led_state = "off"
debug_mode = False  # Set to 'False' to disable debug warnings

def shutdown_callback():
    """ Callback triggered when GPIO_IN detects a rising edge, shutting down the Raspberry Pi. """
    if debug_mode:
        rospy.logwarn(f"⚠️ GPIO {GPIO_IN} detected: Initiating shutdown...")
    subprocess.call(["shutdown", "now"])

def on_shutdown():
    """ Function executed when ROS shuts down the node. It turns off all LEDs before exiting. """
    if debug_mode:
        rospy.logwarn("⚠️ Node is shutting down. Turning off all LEDs.")
    GPIO.output(GPIO_OUT, GPIO.LOW)
    GPIO.output(GPIO_RED, GPIO.LOW)
    GPIO.output(GPIO_GREEN, GPIO.LOW)
    #GPIO.cleanup()

def set_led_state(state):
    """ Turns LEDs on/off based on the given state """
    global blinking, led_state  # Use global variables
    led_state = state
    blinking = False

    if debug_mode:
        rospy.logwarn(f"🔆 LED State Change: {state}")

    if state == "off":
        GPIO.output(GPIO_RED, GPIO.LOW)
        GPIO.output(GPIO_GREEN, GPIO.LOW)
        if debug_mode:
            rospy.logwarn("🟡 LEDs OFF")
    elif state == "ready":
        GPIO.output(GPIO_RED, GPIO.LOW)
        GPIO.output(GPIO_GREEN, GPIO.HIGH)
        if debug_mode:
            rospy.logwarn("🟢 READY: Green LED ON")
    elif state == "recording":
        blinking = True  # Green LED blinking at 1Hz
        if debug_mode:
            rospy.logwarn("🟢 RECORDING: Green LED will blink")
    elif state == "warning":
        GPIO.output(GPIO_RED, GPIO.HIGH)
        GPIO.output(GPIO_GREEN, GPIO.HIGH)
        if debug_mode:
            rospy.logwarn("🟡 WARNING: Both LEDs ON")
    elif state == "error":
        GPIO.output(GPIO_RED, GPIO.HIGH)
        GPIO.output(GPIO_GREEN, GPIO.LOW)
        if debug_mode:
            rospy.logwarn("🔴 ERROR: Red LED ON")
    elif state == "nofix":
        blinking = True  # Both LEDs blinking at 1Hz
        if debug_mode:
            rospy.logwarn("🟡 NOFIX: Both LEDs will blink")
    else:
        if debug_mode:
            rospy.logwarn(f"❌ Invalid LED state received: {state}")

def led_control_callback(msg):
    """ ROS callback to update LED state based on received message """
    if debug_mode:
        rospy.logwarn(f"📩 Received LED state: {msg.data}")
    set_led_state(msg.data)

def main():
    """ Main ROS loop """
    rospy.init_node('gpio_control_node', anonymous=True)

    if debug_mode:
        rospy.logwarn("🚀 Starting GPIO Control Node...")
    
    rospy.on_shutdown(on_shutdown)

    # Subscribe to the topic to receive LED state commands
    rospy.Subscriber("led_control", String, led_control_callback)

    # Initialize GPIO
    try:
        GPIO.setwarnings(False)
        GPIO.cleanup()
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_OUT, GPIO.OUT)
        GPIO.output(GPIO_OUT, GPIO.HIGH)

        GPIO.setup(GPIO_IN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(GPIO_IN, GPIO.FALLING, callback=lambda _: shutdown_callback(), bouncetime=300)

        GPIO.setup(GPIO_Pulse, GPIO.OUT)
        GPIO.setup(GPIO_RED, GPIO.OUT)
        GPIO.setup(GPIO_GREEN, GPIO.OUT)
        GPIO.output(GPIO_RED, GPIO.LOW)
        GPIO.output(GPIO_GREEN, GPIO.LOW)
        GPIO.setwarnings(True)
        if debug_mode:
            rospy.logwarn("✅ GPIO Initialization Successful!")

    except Exception as e:
        if debug_mode:
            rospy.logwarn(f"❌ GPIO Initialization Failed: {e}")
        return

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
                if debug_mode:
                    rospy.logwarn("🟢 RECORDING: Green LED Blinking")
            elif led_state == "nofix":
                GPIO.output(GPIO_RED, not GPIO.input(GPIO_RED))  # Red blinking
                GPIO.output(GPIO_GREEN, not GPIO.input(GPIO_GREEN))  # Green blinking
                if debug_mode:
                    rospy.logwarn("🟡 NOFIX: Both LEDs Blinking")

        rate.sleep()

if __name__ == "__main__":
    main()
