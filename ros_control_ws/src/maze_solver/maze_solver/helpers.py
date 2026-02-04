# ----------------------------
# Helpers
# ----------------------------

def left_cell(current_position, heading):
    return forward_cell(current_position, (heading - 90) % 360)

def right_cell(current_position, heading):
    return forward_cell(current_position, (heading + 90) % 360)

def forward_cell(current_position, heading):
    r, c = current_position
    if heading == 0:     return (r + 1, c)
    if heading == 90:    return (r, c + 1)
    if heading == 180:   return (r - 1, c)
    if heading == 270:   return (r, c - 1)
    return current_position

def leftforward_cell(current_position, heading):
    fwd = forward_cell(current_position, heading)
    return left_cell(fwd, heading)

def rightforward_cell(current_position, heading):
    fwd = forward_cell(current_position, heading)
    return right_cell(fwd, heading)

def backward_cell(current_position, heading):
    return forward_cell(current_position, (heading + 180) % 360)

def backleft_cell(current_position, heading):
    back = backward_cell(current_position, heading)
    return left_cell(back, heading)

def backright_cell(current_position, heading):
    back = backward_cell(current_position, heading)
    return right_cell(back, heading)

def in_bounds(cell, map):
    val = map.get(cell, False)
    return False if val == False else True

def is_free(self, cell):
    map = self.BB.get("map")
    return in_bounds(cell, map) and (map[cell] != "wall") and (map[cell] != "unmapped")

def left_of(heading):    return (heading - 90) % 360
def right_of(heading):   return (heading + 90) % 360
def back_of(heading):    return (heading + 180) % 360

def neighbor_cells_with_headings(current_position, heading):
    return [
        (heading, forward_cell(current_position, heading)),
        (left_of(heading),  forward_cell(current_position, left_of(heading))),
        (right_of(heading), forward_cell(current_position, right_of(heading))),
        (back_of(heading),  forward_cell(current_position, back_of(heading))),
    ]
