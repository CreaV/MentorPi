"""Low front counterweight tray for Anker Prime A1335 or strap-compatible packs.

The power bank lies across the front of the chassis. Its 55 mm dimension is
vertical and the USB-C end remains open. Two straps are mandatory.
"""

from build123d import Align, Box, Cylinder, Location

PACK_LENGTH_Y = 134.4
PACK_DEPTH_X = 34.53
PACK_HEIGHT_Z = 55.0
CLEARANCE = 0.8
WALL = 3.0
FLOOR = 3.0


def slot_y(length: float, width: float, height: float):
    body = Box(width, length - width, height, align=(Align.CENTER, Align.CENTER, Align.MIN))
    end = Cylinder(width / 2, height, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return body + end.moved(Location((0, (length - width) / 2, 0))) + end.moved(
        Location((0, -(length - width) / 2, 0))
    )


def gen_step():
    inner_x = PACK_DEPTH_X + 2 * CLEARANCE
    outer_x = inner_x + 2 * WALL
    outer_y = PACK_LENGTH_Y + 2 * CLEARANCE + 2 * WALL
    tray = Box(outer_x, outer_y, FLOOR, align=(Align.CENTER, Align.CENTER, Align.MIN))
    wall_x = inner_x / 2 + WALL / 2
    for x in (-wall_x, wall_x):
        tray += Box(WALL, outer_y, 18.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((x, 0, FLOOR))
        )
    # Low end-stop opposite the USB-C end; the other short end remains open.
    tray += Box(outer_x, WALL, 9.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, -outer_y / 2 + WALL / 2, FLOOR))
    )
    for y in (-38.0, 38.0):
        tray -= slot_y(24.0, 5.0, FLOOR + 2).moved(Location((0, y, -1)))

    # Twin hooks hang the tray from the front top edge. Clamp with M4 screws
    # through the holes after a dry-fit; the 5 mm throat tolerates the STL-only
    # chassis edge until it is measured on the physical robot.
    rear_x = -outer_x / 2
    for y in (-43.0, 43.0):
        tab = Box(8.0, 18.0, 58.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((rear_x - 4.0, y, 0))
        )
        lip = Box(15.0, 18.0, 4.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((rear_x - 0.5, y, 54.0))
        )
        tray += tab + lip
        screw = Cylinder(2.25, 10.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((rear_x - 4.0, y, 44.0), (0, 90, 0))
        )
        tray -= screw
    tray.label = "anker_prime_a1335_low_front_tray"
    return tray

