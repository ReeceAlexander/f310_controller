import pygame
import time

# Initialize Pygame and Joystick
pygame.init()
pygame.joystick.init()

# Check if there’s a joystick connected
if pygame.joystick.get_count() == 0:
    print("No joystick connected!")
    pygame.quit()
    exit()

# Use the first connected joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Connected to joystick: {joystick.get_name()}")

try:
    while True:
        # Process events
        pygame.event.pump()

        # Get axis values
        for i in range(joystick.get_numaxes()):
            axis_value = joystick.get_axis(i)
            print(f"Axis {i}: {axis_value:.2f}")

        # Get button values
        for i in range(joystick.get_numbuttons()):
            button_value = joystick.get_button(i)
            print(f"Button {i}: {'Pressed' if button_value else 'Released'}")

        # Get hat (D-pad) values
        for i in range(joystick.get_numhats()):
            hat_value = joystick.get_hat(i)
            print(f"Hat {i}: {hat_value}")

        # Small delay to reduce console spam
        time.sleep(0.1)
        print("\n" + "-"*20 + "\n")

except KeyboardInterrupt:
    print("Exited.")

finally:
    pygame.joystick.quit()
    pygame.quit()
