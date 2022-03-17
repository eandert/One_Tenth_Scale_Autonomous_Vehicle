import sys
import config
from connected_autonomous_vehicle.src import cav

conf = config.Setting("two_cav_physical")

cav.cav(conf, 0)
