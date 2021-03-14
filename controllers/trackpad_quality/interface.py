import pygame

class Interface:
    def __init__(self, size):
        pygame.init()

        pygame.display.set_caption("Trackpad Input")
        self.window_surface = pygame.display.set_mode((size, size))

        self.cursor = pygame.Rect(0, 0, 0, 0)

    def show_input(self, x, y):
        size = 4
        self.cursor.update(x-(size/2), y-(size/2), size, size)
        colour = pygame.Color("#FAFAFC")

        self.window_surface.fill(pygame.Color("#000000"))
        pygame.draw.rect(self.window_surface, colour, self.cursor)
        pygame.display.flip()

        print(x, y)
