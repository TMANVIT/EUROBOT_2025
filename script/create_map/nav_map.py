import cv2
import numpy as np

# map_size(in pixels)
map_img = np.ones((420, 620), dtype=np.uint8) * 255  # white background

# decks coordinates(in pixels)
decks = [
    [(15, 370), (35, 290)],                 #left_upper_material

    [(15, 185), (35, 105)],                 #left_lower_material

    [(605, 370), (585, 290)],               #right_upper_material

    [(605, 185), (585, 105)],               #right_lower_material

    [(310-40, 210), (310-120, 210+20)],     #center_material

    [(310+40, 210), (310+120, 210+20)],     #center_material

    [(125, 370), (205, 350)],               #lower_material

    [(495, 370), (415, 350)],               #lower_material

    [(10, 10), (610, 410)],                 #borders

    [(140,50), (220,10)],                   #ramp

    [(480,50), (400,10)],                   #ramp

    [(400, 10), (220, 100)],                #stage

    [(310-95, 55), (310-175, 75)],          #stage_material

    [(310+95, 55), (310+175, 75)]           #stage_material
]


# decks drawing
for i in decks:
    cv2.rectangle(map_img, i[0], i[1],(0, 0, 0), 1)

# Saving pgm file
cv2.imwrite("nav_map.pgm", map_img)