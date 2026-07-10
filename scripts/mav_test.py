from mavlink_controller import MavController
import time

SERIAL_PORT = "/dev/tty.usbmodem1201"
SERIAL_BAUD = 115200

if __name__ == "__main__":
    # Init MavController first
    print("Initializing MavController...")
    mav_controller = MavController(port=SERIAL_PORT, baud=SERIAL_BAUD)
    
    while not mav_controller.is_connected:
        print("Waiting for connection...")
        time.sleep(1)

    while True:
        choice = input('''Select an option:
0: exit program
1: print status
2: arm/disarm
3: 
Enter your choice: ''').strip()

        if choice == "0":
            print("Exiting program.")
            break
        elif choice == "1":
            status = mav_controller.get_status()
            print(f"Status: {status}\n-EOF")
        elif choice == "2":
            if mav_controller.is_armed:
                print("Disarming...")
                mav_controller.disarm()
            else:
                print("Arming...")
                mav_controller.arm()
        else:
            print("Invalid choice. Exiting.")
            break
