# position of the center ROI
roi_center_x = 420
roi_center_y = 230

# colors in BGR
colors = [('green',  (65,135,25)),
          ('orange', (30,75,250)),
          ('white',  (150,155,160)),
          ('blue',   (125,75,0)),
          ('red',    (40, 20, 180)),
          ('yellow', (25,165,175))]

color2index = {
    c[0]: i for i, c in enumerate(colors)
}


index2color = {
    i: c[0] for i, c in enumerate(colors)
}
