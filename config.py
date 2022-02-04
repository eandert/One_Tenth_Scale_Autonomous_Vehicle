''' This is not the prettyiest way to do this but this class allows 
for the definition and slection of multiple simulation/physical
settings for the 1/10 scale setup '''
class Setting:
        def __init__(self, setting = "two_cav_simulation"):
                if "two_cav_physical" in setting:
                        print(" Config: two_cav_physical selected")
                        # Working 2 CAV Real
                        self.cav = [[-0.75, 0.0, 0, False],
                        [-1.5, 0.0, 0, False]]
                        #       [0.0, 0.75, 4.71238898038469, False]]
                        self.cis = [[-0.75, 0.75, -45, False]]
                        self.rsu_ip = '192.168.0.103' #'127.0.0.1'
                        self.interval = 0.125
                        self.offset_interval = 0.0
                        self.fallthrough_delay = 0.100
                        self.init_time = 10.0 # Seconds to wait for the system to initialize before starting
                        self.map = 0
                        self.map_length = 0.5
                        self.simulation = False
                        self.debug = True
                        self.unit_test = False
                        self.unit_test_config = None
                        self.gui_interval = 1000 # 1 hz to slow down the toll on the RSU
                elif "four_cav_simulation" in setting:
                        print("         Config: four_cav_simulation selected")
                        # Working 2 CAV simulation
                        self.cav = [[-0.75, 0.0, 0, True],
                                [-1.5, 0.0, 0, True],
                                [0.0, 0.75, 4.71238898038469, True],
                                [0.0, 1.75, 4.71238898038469, True]]
                        self.cis = [[-0.75, 0.75, -45, True]]
                        self.rsu_ip = '127.0.0.1'
                        self.interval = 0.125
                        self.offset_interval = 0.0
                        self.fallthrough_delay = 0.100
                        self.init_time = 10.0 # Seconds to wait for the system to initialize before starting
                        self.map = 0
                        self.map_length = 0.5
                        self.simulation = True
                        self.debug = True
                        self.data_collect_mode = False
                        self.unit_test = False
                        self.unit_test_config = None
                        self.gui_interval = 100 # 10 hz, hopefully this is fast enough
                else:
                        print("         Config: two_cav_simulation selected")
                        # Working 2 CAV simulation
                        self.cav = [[-0.75, 0.0, 0, True],
                                [0.0, 0.75, 4.71238898038469, True]]
                        self.cis = [[-0.75, 0.75, -45, True]]
                        self.rsu_ip = '127.0.0.1'
                        self.interval = 0.125
                        self.offset_interval = 0.0
                        self.fallthrough_delay = 0.100
                        self.init_time = 10.0 # Seconds to wait for the system to initialize before starting
                        self.map = 0
                        self.map_length = 0.5
                        self.simulation = True
                        self.debug = True
                        self.data_collect_mode = False
                        self.unit_test = False
                        self.unit_test_config = None
                        self.gui_interval = 100 # 10 hz, hopefully this is fast enough
                print(" Config complete, using map: ", self.map)