import pygame

# Initialize pygame and the joystick module
pygame.init()
pygame.joystick.init()

# Check for connected controllers
if pygame.joystick.get_count() == 0:
    print("No EdgeTX radio found. Ensure it is in USB Joystick mode.")
    exit()

# Initialize the first joystick (the EdgeTX radio)
radio = pygame.joystick.Joystick(1)
radio.init()

print(f"Connected to: {radio.get_name()}")

try:
    while True:
        # Standard Pygame event pump to update stick states
        pygame.event.pump()
        
        # Read Axes (Sticks: Aileron, Elevator, Throttle, Rudder)
        # Values typically range from -1.0 to 1.0
        axes = [radio.get_axis(i) for i in range(radio.get_numaxes())]
        
        # Read Buttons (Switches mapped as buttons)
        buttons = [radio.get_button(i) for i in range(radio.get_numbuttons())]
        
        print(f"Axes: {[round(a, 2) for a in axes]} | Buttons: {buttons}", end="\r")

except KeyboardInterrupt:
    pygame.quit()
