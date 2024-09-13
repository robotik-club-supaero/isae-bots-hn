
def adapt_pos_to_side(x, y, theta, color):
    if color == 0:
        return x, y, theta
    else:
        return x, 3000-y, -theta        # si la symétrie est selon l'axe y
        # return 2000-x, y, theta + pi  # si la symétrie est selon l'axe x
     
