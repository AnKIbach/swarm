#!/usr/bin/env python
import sys
import pygame
from Setup_simulator import *

pygame.init()
clock = pygame.time.Clock()

fps = 10
ani = 4
window = Display(800, 600) #setter displayet
gameWindow = pygame.display.set_mode((window.width, window.height))

boat = Boat() #spawner en båt
boat.rect.x = 0
boat.rect.y = 0
boat_list = pygame.sprite.Group()
boat_list.add(boat)

loop = True



while loop == True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit(); sys.exit()
            loop = False

        if event.type == pygame.KEYDOWN:
            if event.key == ord('q'):
                pygame.quit()
                sys.exit()
                loop = False
    
    boat_list.draw(gameWindow)
    pygame.display.flip()
    clock.tick(fps)