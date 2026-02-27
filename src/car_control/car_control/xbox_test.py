import pygame
import sys

pygame.init()
pygame.joystick.init()

# Check for controller
if pygame.joystick.get_count() == 0:
    print("No controller detected")
    sys.exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Controller connected:", joystick.get_name())

# Create window
screen = pygame.display.set_mode((600, 400))
pygame.display.set_caption("Controller Test")

font = pygame.font.Font(None, 30)

clock = pygame.time.Clock()

while True:
    pygame.event.pump()

    screen.fill((30, 30, 30))

    y_offset = 20

    # AXES
    for i in range(joystick.get_numaxes()):
        axis = joystick.get_axis(i)
        text = font.render(f"Axis {i}: {axis:.3f}", True, (255, 255, 255))
        screen.blit(text, (20, y_offset))
        y_offset += 30

    # BUTTONS
    for i in range(joystick.get_numbuttons()):
        button = joystick.get_button(i)
        text = font.render(f"Button {i}: {button}", True, (255, 255, 255))
        screen.blit(text, (300, 20 + i * 30))

    pygame.display.flip()
    clock.tick(60)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()


