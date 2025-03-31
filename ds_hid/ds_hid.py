import usb.core
import usb.util
import usb.backend.libusb1
from usbmonitor import USBMonitor
import vgamepad as vg
import keyboard as kb
import json


class RobotConfig:
    V1_STACKUP = "./v1_stackUp_bindings.json"
    V2_REDUNDANCY = "./v2_redundancy_bindings.json"

# XK-80 Vendor and Product ID
VENDOR_ID = 1523
PRODUCT_ID = 1091

# Robot Config
ROBOT = RobotConfig.V1_STACKUP

print("""

███████╗██████╗░░█████╗░░░███╗░░░█████╗░░█████╗░    ██████╗░░█████╗░░█████╗░████████╗
██╔════╝██╔══██╗██╔══██╗░████║░░██╔══██╗██╔══██╗    ██╔══██╗██╔══██╗██╔══██╗╚══██╔══╝
█████╗░░██████╔╝██║░░╚═╝██╔██║░░╚██████║██║░░██║    ██████╔╝██║░░██║██║░░██║░░░██║░░░
██╔══╝░░██╔══██╗██║░░██╗╚═╝██║░░░╚═══██║██║░░██║    ██╔═══╝░██║░░██║██║░░██║░░░██║░░░
██║░░░░░██║░░██║╚█████╔╝███████╗░█████╔╝╚█████╔╝    ██║░░░░░╚█████╔╝╚█████╔╝░░░██║░░░
╚═╝░░░░░╚═╝░░╚═╝░╚════╝░╚══════╝░╚════╝░░╚════╝░    ╚═╝░░░░░░╚════╝░░╚════╝░░░░╚═╝░░░

░█████╗░░█████╗░███╗░░██╗████████╗██████╗░░█████╗░██╗░░░░░██╗░░░░░███████╗██████╗░
██╔══██╗██╔══██╗████╗░██║╚══██╔══╝██╔══██╗██╔══██╗██║░░░░░██║░░░░░██╔════╝██╔══██╗
██║░░╚═╝██║░░██║██╔██╗██║░░░██║░░░██████╔╝██║░░██║██║░░░░░██║░░░░░█████╗░░██████╔╝
██║░░██╗██║░░██║██║╚████║░░░██║░░░██╔══██╗██║░░██║██║░░░░░██║░░░░░██╔══╝░░██╔══██╗
╚█████╔╝╚█████╔╝██║░╚███║░░░██║░░░██║░░██║╚█████╔╝███████╗███████╗███████╗██║░░██║
░╚════╝░░╚════╝░╚═╝░░╚══╝░░░╚═╝░░░╚═╝░░╚═╝░╚════╝░╚══════╝╚══════╝╚══════╝╚═╝░░╚═╝

██╗███╗░░██╗████████╗███████╗██████╗░███████╗░█████╗░░█████╗░███████╗
██║████╗░██║╚══██╔══╝██╔════╝██╔══██╗██╔════╝██╔══██╗██╔══██╗██╔════╝
██║██╔██╗██║░░░██║░░░█████╗░░██████╔╝█████╗░░███████║██║░░╚═╝█████╗░░
██║██║╚████║░░░██║░░░██╔══╝░░██╔══██╗██╔══╝░░██╔══██║██║░░██╗██╔══╝░░
██║██║░╚███║░░░██║░░░███████╗██║░░██║██║░░░░░██║░░██║╚█████╔╝███████╗
╚═╝╚═╝░░╚══╝░░░╚═╝░░░╚══════╝╚═╝░░╚═╝╚═╝░░░░░╚═╝░░╚═╝░╚════╝░╚══════╝

""")
print("The Poot Controller Interface by FRC Team 190 is used to interface with the XK-80 Keyboard using emulated Xbox 360 controllers. \n Thanks to team 204 for the basic setup! \n")
print("usage: [right shift] to toggle, close this window to stop the emulator")
print("\nstarting..")

active = False

def flipActive():
    global active
    active = not active
    print("enabled" if active else "disabled")

kb.add_hotkey('right shift', flipActive)

def load_key_mapping():
    with open(ROBOT, "r") as f:
        return json.load(f)

key_mapping = load_key_mapping()

# Create multiple virtual gamepads
controllers = {subsystem: vg.VX360Gamepad() for subsystem in key_mapping.keys()}

connected = False

def on_disconnect(device_id, device_info):
    global connected
    connected = False
    print(f"Device disconnected: {device_id}")

def on_connect(device_id, device_info):
    global connected
    connected = True
    print(f"Device connected: {device_id}")

monitor = USBMonitor()
monitor.start_monitoring(on_connect=on_connect, on_disconnect=on_disconnect, check_every_seconds=0.02)

# Find the XK-80 device
backend = usb.backend.libusb1.get_backend()
device = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID, backend=backend)

if device is None:
    print("XK-80 not found! Check connections.")
    exit(1)

print("XK-80 found and ready!")

# Detach kernel driver if necessary (Linux only)
if device.is_kernel_driver_active(0):
    device.detach_kernel_driver(0)

# Set device configuration
device.set_configuration()

# Find the HID endpoint for reading input
cfg = device.get_active_configuration()
interface = cfg[(0, 0)]
endpoint = usb.util.find_descriptor(
    interface,
    custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
)

if endpoint is None:
    print("No IN endpoint found on XK-80")
    exit(1)

print("Listening for XK-80 key presses...")

def process_key_presses(pressed_keys):
    if not active:
        return
    
    for subsystem, mapping in key_mapping.items():
        gamepad = controllers[subsystem]
        
        for key, action in mapping.items():
            if int(key) in pressed_keys:
                if "button" in action:
                    gamepad.press_button(getattr(vg.XUSB_BUTTON, action["button"]))
                if "axis" in action:
                    if action["axis"] == "left_x":
                        gamepad.left_joystick_float(action["value"], 0.0)
                    elif action["axis"] == "left_y":
                        gamepad.left_joystick_float(0.0, action["value"])
                    elif action["axis"] == "right_x":
                        gamepad.right_joystick_float(action["value"], 0.0)
                    elif action["axis"] == "right_y":
                        gamepad.right_joystick_float(0.0, action["value"])
                if "trigger" in action:
                    if action["trigger"] == "left":
                        gamepad.left_trigger_float(action["value"])
                    elif action["trigger"] == "right":
                        gamepad.right_trigger_float(action["value"])
            else:
                if "button" in action:
                    gamepad.release_button(getattr(vg.XUSB_BUTTON, action["button"]))
                if "axis" in action:
                    if action["axis"] in ["left_x", "left_y"]:
                        gamepad.left_joystick_float(0.0, 0.0)
                    if action["axis"] in ["right_x", "right_y"]:
                        gamepad.right_joystick_float(0.0, 0.0)
                if "trigger" in action:
                    if action["trigger"] == "left":
                        gamepad.left_trigger_float(0.0)
                    elif action["trigger"] == "right":
                        gamepad.right_trigger_float(0.0)
        
        gamepad.update()

try:
    while True:
        try:
            if connected:
                data = device.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize, timeout=5000)
                
                if data:
                    key_states = data[2:12]  # 10 bytes, representing 10 columns
                    pressed_keys = []

                    for col_index, byte in enumerate(key_states):
                        for row_index in range(8):
                            if (byte >> row_index) & 1:
                                key_number = (col_index) * 8 + row_index
                                pressed_keys.append(key_number)

                    process_key_presses(pressed_keys)
        
        except usb.core.USBTimeoutError:
            pass  # Ignore timeout errors

        except usb.core.USBError as e:
            if e.errno == 5:  # Input/Output Error = Disconnected device
                print("Device disconnected!")
                pass

except KeyboardInterrupt:
    print("\nStopping script.")

finally:
    print("Cleaning up...")
    monitor.stop_monitoring()

    try:
        usb.util.release_interface(device, 0)
        device.attach_kernel_driver(0)  # Only needed on Linux
    except:
        pass  # Ignore errors if device is already gone
