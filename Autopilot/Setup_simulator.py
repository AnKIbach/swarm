#!/usr/bin/env python
import pygame 
import sys
import os

class Display:
    def __init__(self, display_width = 200, display_height = 100):
        self.width = display_width
        self.height = display_height
        self.background = (0,0,0)
        pygame.display.set_caption('Kim & Andreas sin baot simerer')

class Boat(pygame.sprite.Sprite):
    #skape en båt
    def __init__(self):
        super(Boat, self).__init__()
        self.move_x = 0
        self.move_y = 0
        self.frame = 0
        self.images = []
        img = pygame.image.load('source_pictures/boatTest.png') #bilet ligger i annen mappe
        self.images.append(img)
        self.image = self.images[0]
        self.rect = self.image.get_rect()

    def control(self,x,y):
        #bevegelse av båten
        self.move_x += x
        self.move_y += y

    def update(self):
        #oppdatere posisjon til båten
        self.rect.x = self.rect.x + self.move_x
        self.rect.y = self.rect.y + self.move_y


