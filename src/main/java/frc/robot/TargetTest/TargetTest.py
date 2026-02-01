import pygame # Using Pygame because you dont have to mess with PyImGui
from networktables import NetworkTables # Network tables is what the robot publishes info and grabs in this case too

# ------------------ CONFIG ------------------
ROBOT_IP = "10.XX.YY.2"   # change to your team IP
FIELD_IMAGE_PATH = "FieldImage.png"

FIELD_LENGTH = 16.54  # meters
FIELD_WIDTH  = 8.07   # meters
# --------------------------------------------
last_click = None  # (px, py)

# NetworkTables init
NetworkTables.initialize(server="127.0.0.2") # Sim with 127.0.0.2 Normal is 10.TE.AM.2 will eventually implement
nt = NetworkTables.getTable("dashboard") # Network Table ID

# Pygame init
pygame.init()
field_image = pygame.image.load(FIELD_IMAGE_PATH)

WIDTH, HEIGHT = field_image.get_size()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("FRC 2026 Field Dashboard")

clock = pygame.time.Clock()

def pixel_to_field(px, py):
    x_meters = (px / WIDTH) * FIELD_LENGTH
    y_meters = FIELD_WIDTH - (py / HEIGHT) * FIELD_WIDTH
    return x_meters, y_meters

running = True
while running:
    screen.blit(field_image, (0, 0))

    # draw persistent marker
    if last_click is not None:
        pygame.draw.circle(screen, (255, 0, 0), last_click, 6)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.MOUSEBUTTONDOWN:
            px, py = pygame.mouse.get_pos()
            last_click = (px, py)

            x, y = pixel_to_field(px, py)

            nt.putNumber("targetX", x)
            nt.putNumber("targetY", y)

            print(f"Sent target: X={x:.2f}m  Y={y:.2f}m")

    pygame.display.flip()
    clock.tick(60)


pygame.quit()
