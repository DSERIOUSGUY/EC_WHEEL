from controller import Robot, Supervisor
import os
import numpy as np
import pygame

class Monitor(Supervisor):
    def __init__(self, num_of_nodes, threshold, timestep):
        super().__init__()
        self.nodes = []
        self.init_mesh(num_of_nodes)
        self.init_display()
        self.threshold = threshold
        self.wheelchair = self.getFromDef("CHAIR_FULL")
        self.TIME_STEP = timestep
        main_dir = os.path.split(os.path.abspath(__file__))[0]
        self.background = pygame.image.load(os.path.join(main_dir, "home.png")).convert()
        
        
    def init_mesh(self, num_of_nodes):
        for i in range(1, num_of_nodes+1):
            self.nodes.append(self.getFromDef("esp%d" % i))
            print(self.nodes[i-1].getDef())
            
    def init_display(self):
        pygame.init()

        pygame.display.set_caption("Wheelchair Monitor")
        self.screen = pygame.display.set_mode((750, 550))
        
            
    def dist_bet_nodes(self, node1, node2):
        return np.linalg.norm(np.array(node1.getPosition()) - np.array(node2.getPosition()))

    def get_can_see(self):
        can_see = []
        for node in self.nodes:
            if (self.dist_bet_nodes(node, self.wheelchair) < self.threshold):
                can_see.append(node)
                print(node.getDef())
        return can_see
    
    def show_input(x, y):
        size = 4
        self.cursor.update(x-(size/2), y-(size/2), size, size)
        colour = pygame.Color("#FAFAFC")
    
        self.window_surface.fill(pygame.Color("#000000"))
        pygame.draw.rect(self.window_surface, colour, self.cursor)
        pygame.display.flip()

        print(x, y)
    
    def go(self):
        while self.step(self.TIME_STEP) != -1:
            self.screen.blit(self.background, (0, 0))
            can_see = self.get_can_see()
            for node in self.nodes:
                # print()
                # print(node.getDef())
                # print(node.getPosition())
                pos = (node.getPosition()[0] * 47 + 375, node.getPosition()[2] * 35 + 275)
                # print(pos)
                
                col = "black"
                if node in can_see:
                    # print("seen")
                    col = "red"
                pygame.draw.circle(self.screen, pygame.Color(col), pos, 10)
            pygame.display.flip()

mon = Monitor(8, 4, 64)

mon.go()
    


