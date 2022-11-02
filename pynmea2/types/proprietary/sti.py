# pylint: disable=wildcard-import,unused-wildcard-import
from decimal import Decimal

from ... import nmea
from ...nmea_utils import *


# SkyTraq
class STI(nmea.ProprietarySentence):
    sentence_types = {}

    def __new__(_cls, manufacturer, data):
        name = manufacturer + data[1]
        cls = _cls.sentence_types.get(name, _cls)
        return super(STI, cls).__new__(cls)


class STI030(STI, LatLonFix):
    """ Minimum recommended 3D DNSS Data
    """
    fields = (
        ("Blank", "_blank"),
        ("STI Type", "sti_type"),
        ("Time of fix","time", timestamp),
        ("Status", "fix_status"),
        ("Latitude", "lat"),
        ("Latitude Direction", "lat_dir"),
        ("Longitude", "lon"),
        ("Longitude Direction", "lon_dir"),
        ("MSL","altitude", float),
        ("East velocity", "east_velocity", float),
        ("North velocity", "north_velocity", float),
        ("Vertical velocity", "vert_velocity", float),
        ("UTC Date", "date", datestamp),
        ("Mode", "mode"),
        ("RTK Age", "age", float),
        ("RTK Ratio", "ratio", float),
    )

class STI032(STI):
    """ RTK Baseline Data
    """

    fields = (
        ("Blank", "_blank"),
        ("STI Type", "sti_type"),
        ("Time of fix","time", timestamp),
        ("UTC Date", "date", datestamp),
        ("Status", "fix_status"),
        ("Mode", "mode"),
        ("East projection from baseline", "east", float),
        ("North projection from baseline", "north", float),
        ("Elevation from baseline", "elevation", float),
        ("Distance from baseline", "dist", float),
        ("Bearing from baseline", "bearing", float),
    )

class STI035(STI):
    """ RTK Baseline data of rover moving base antenna
    """
    fields = (
        ("Blank", "_blank"),
        ("STI Type", "sti_type"),
        ("Time of fix","time", timestamp),
        ("UTC Date", "date", datestamp),
        ("Status", "fix_status"),
        ("Mode", "mode"),
        ("East projection from moving base", "east", float),
        ("North projection from moving base", "north", float),
        ("Elevation from moving base", "elevation", float),
        ("Distance from moving base", "dist", float),
        ("Bearing from moving base", "bearing", float),
    )


class STI036(STI):
    """ Heading, Pitch, or roll of Vehicle
    """

    fields = (
        ("Blank", "_blank"),
        ("STI Type", "sti_type"),
        ("Time of fix","time", timestamp),
        ("UTC Date", "date", datestamp),
        ("Heading", "heading", float),
        ("Pitch", "pitch", float),
        ("Roll", "roll", float),
        ("Mode", "mode"),
    )



   


