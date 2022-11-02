# pylint: disable=wildcard-import,unused-wildcard-import
from decimal import Decimal

from ... import nmea
from ...nmea_utils import *


# BNO 08x
class BNO(nmea.ProprietarySentence):
    sentence_types = {}

    def __new__(_cls, manufacturer, data):
        name = manufacturer + data[1]
        cls = _cls.sentence_types.get(name, _cls)
        return super(BNO, cls).__new__(cls)


class BNO01(BNO):
    """ Minimum recommended 3D DNSS Data
    """
    fields = (
        ("Blank", "_blank"),
        ("BNO message type", "bno_type"),
        ("Timestamp", "timestamp", float),
        ("Roll", "roll", float),
        ("Pitch", "pitch", float),
        ("Yaw", "yaw", float),
    )

