# position of the center ROI
roi_center_x = 420
roi_center_y = 230

# colors in BGR
colors = [('green',  (0, 60, 0)),
          ('orange', (0, 20, 90)),
          ('white',  (60, 60, 60)),
          ('blue',   (30, 20, 0)),
          ('red',    (0, 0, 70)),
          ('yellow', (0, 56, 79))]

color2index = {
    c[0]: i for i, c in enumerate(colors)
}


index2color = {
    i: c[0] for i, c in enumerate(colors)
}
