import serial
from time import sleep
import keyboard # Requires root/admin access to run, relevant libraries need installation on root

""" Moves steppers taking correct formulated serial command as string (f"{Motor1Steps} {Motor2Steps} {resetPosBool}")"""
def move(message):
    message += '\n'
    ser.write(message.encode('ascii'))
    #ser.flush()

serialName: str = '/dev/ttyACM0' #Linux port name
baud: int = 19200
ser = serial.Serial(serialName, baudrate=baud)#, rtscts=False, dsrdtr=False)  # open serial port
sleep(1)

if __name__ == '__main__':
    #while(True):
         #message = input()
         #if message == 'stop':
             #break
         #move(message)

    #region Keyboard
    # Keyboard måste köras som root: sudo pycharm-community
    # Alla libraries måste installeras på root python också
    Mstep1: int = 0
    Mstep2: int = 0

    def on_key_event(event):
        global Mstep1
        global Mstep2
        if event.event_type == "down":
            #print(f"Key {event.name} pressed")
            if event.name not in ["up","down","right","left"]:
                #print("not move key")
                return
            if event.name == "up":
                Mstep1 += 1
            if event.name == "down":
                Mstep1 -= 1
            if event.name == "right":
                Mstep2 += 1
            if event.name == "left":
                Mstep2 -= 1

            print(f"{Mstep1} {Mstep2} 0")
            move(f"{Mstep1} {Mstep2} 0")

    keyboard.hook(on_key_event)
    # This will block the script and wait until 'esc' is pressed
    keyboard.wait("esc")

    #endregion


# funktion vinklar till rörelse
# Hur snabbt kan man skicka pos utan att det ballar ur?
# Om man skickar ny pos avslutas tidigare rörelse och ny rörelse fullföljs till korrekt steg

