# position of the center ROI
roi_center_x = 420
roi_center_y = 230

# colors in BGR
colors = [('green',  (96, 178, 60)),
          ('orange', (30, 100, 255)),
          ('white',  (175, 175, 185)),
          ('blue',   (153, 102, 0)),
          ('red',    (20, 5, 200)),
          ('yellow', (80, 190, 253))]

color2index = {
    c[0]: i for i, c in enumerate(colors)
}


index2color = {
    i: c[0] for i, c in enumerate(colors)
}
