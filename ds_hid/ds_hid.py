import usb.core
import usb.util
import usb.backend.libusb1
from usbmonitor import USBMonitor
from networktables import NetworkTables

# XK-80 Vendor and Product ID
VENDOR_ID = 1523
PRODUCT_ID = 1091

# Networktables stuff
SERVER = '10.1.90.2'
KEYBOARD = 0
KEYS = 80

connected = False
# Set up NetworkTables connection
try:
    NetworkTables.initialize(server=SERVER) 
except Exception as e:
    print(f'Failed to initialize NetworkTables with server {SERVER}: {e}.')

keyboard_status_table = NetworkTables.getTable('AdvantageKit/DriverStation/Keyboard'+str(KEYBOARD))

def on_disconnect(device_id, device_info):
    keyboard_status_table.putBoolean('isConnected', False)
    connected = False
    print(f"Device disconnected: {device_id}")

def on_connect(device_id, device_info):
    keyboard_status_table.putBoolean('isConnected', True)
    connected = True
    print(f"Device connected: {device_id}")


monitor = USBMonitor()
monitor.start_monitoring(on_connect=on_connect, on_disconnect=on_disconnect, check_every_seconds=0.02)

def send_data(data):
    keyboard_status_table.putBoolean('isConnected', True)
    for key in data:
        keyboard_status_table.putBoolean(str(key), True)

def set_up_NT():
    for i in range(KEYS):
        keyboard_status_table.putBoolean(str(i), False)

set_up_NT()


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
    keyboard_status_table.putBoolean('isConnected', False)
    exit(1)

print("Listening for XK-80 key presses...")


try:
    while True:
        try:
            if connected is True:
                keyboard_status_table.putBoolean('isConnected', True)
                pass
            data = device.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize, timeout=5000)

            if data:
                keyboard_status_table.putBoolean('isConnected', True)
                print(f"Raw Data: {list(data)}")

                key_states = data[2:12]  # 10 bytes, representing 10 columns
                pressed_keys = []

                for col_index, byte in enumerate(key_states):
                    for row_index in range(8):
                        if (byte >> row_index) & 1:
                            key_number = (col_index) * 8 + row_index
                            pressed_keys.append(key_number)

                set_up_NT()
                if pressed_keys:
                    print(f"Keys Pressed: {pressed_keys}")
                    send_data(pressed_keys)
                else:
                    print("No keys pressed.")
            else:
                print("No data received.")
                keyboard_status_table.putBoolean('isConnected', False)

        except usb.core.USBTimeoutError:
            pass  # Ignore timeout errors

        except usb.core.USBError as e:
            if e.errno == 5:  # Input/Output Error = Disconnected device
                print("Device disconnected! Exiting loop...")
                keyboard_status_table.putBoolean('isConnected', False)
                break  # Exit the loop when the device disconnects

except KeyboardInterrupt:
    print("\nStopping script.")

finally:
    print("Cleaning up...")
    monitor.stop_monitoring()
    keyboard_status_table.putBoolean('isConnected', False)
    NetworkTables.stopClient()

    try:
        usb.util.release_interface(device, 0)
        device.attach_kernel_driver(0)  # Only needed on Linux
    except:
        pass  # Ignore errors if device is already gone
