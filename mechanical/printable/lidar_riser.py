"""Universal 170 mm MS200 riser with slotted M3 interfaces."""

from build123d import Align, Box, Cylinder, Location

PLATE_X = 62.0
PLATE_Y = 54.0
PLATE_Z = 5.0
RISE = 170.0


def slot_x(length: float, width: float, height: float):
    body = Box(length - width, width, height, align=(Align.CENTER, Align.CENTER, Align.MIN))
    end = Cylinder(width / 2, height, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return body + end.moved(Location(((length - width) / 2, 0, 0))) + end.moved(
        Location((-(length - width) / 2, 0, 0))
    )


def gen_step():
    bottom = Box(PLATE_X, PLATE_Y, PLATE_Z, align=(Align.CENTER, Align.CENTER, Align.MIN))
    top = Box(PLATE_X, PLATE_Y, PLATE_Z, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, RISE))
    )
    columns = None
    for y in (-20.0, 20.0):
        column = Box(12.0, 10.0, RISE, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((-22.0, y, PLATE_Z))
        )
        columns = column if columns is None else columns + column
    riser = bottom + top + columns
    for z in (-1.0, RISE - 1.0):
        for y in (-16.0, 16.0):
            riser -= slot_x(18.0, 3.6, PLATE_Z + 2).moved(Location((0, y, z)))
    riser.label = "mentorpi_ms200_170mm_riser"
    return riser
