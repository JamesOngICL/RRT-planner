import pygame
import time
pygame.init()
screen = pygame.display.set_mode((800, 600))  # Adjust the size as needed
pygame.display.set_caption("RRT Path Planning")
clock = pygame.time.Clock()
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)

FRAME_RATE = 60  # Adjust as needed
screen.fill(WHITE)  # Clear the screen
time.sleep(0.75)

bg_img = pygame.image.load('Images/map.jpg')
bg_img = pygame.transform.scale(bg_img,(800, 600))
screen.blit(bg_img,(0,0))
# Drawing Rectangle
pygame.draw.rect(screen, BLUE, pygame.Rect(30, 200, 60, 60))
pygame.display.flip()

pygame.display.update()
time.sleep(0.75)
pygame.draw.circle(screen,color=BLUE,center=(400,400),radius=100)
pygame.display.update()
time.sleep(0.75)

pygame.draw.line(screen,color=WHITE,start_pos=(50,50),end_pos=(200,200),width=3)
pygame.display.update()
time.sleep(0.75)


if __name__ =="__main__":
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            continue
    
