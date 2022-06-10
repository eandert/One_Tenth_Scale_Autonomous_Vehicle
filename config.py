''' This is not the prettyiest way to do this but this class allows 
for the definition and slection of multiple simulation/physical
settings for the 1/10 scale setup '''
class Setting:
        def __init__(self, setting = "two_cav_simulation"):
                if "two_cav_physical" == setting:
                        print(" Config: two_cav_physical selected")
                        # Working 2 CAV Real
                        self.cav = [[-0.75, 0.0, 0, False],
                        [-1.5, 0.0, 0, False]]
                        self.cis = [[-0.75, 0.75, -0.785398163, False]]
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
                        self.gui_interval = 100 # 1 hz to slow down the toll on the RSU
                        self.data_collect_mode = True
                        self.use_global_fusion = False
                        self.cooperative_monitoring = False
                        self.test_one_step_kalman = False
                elif "four_cav_simulation" == setting:
                        print("         Config: four_cav_simulation selected")
                        # Working 4 CAV simulation
                        self.cav = [[-0.75, 0.0, 0, True],
                                [-1.5, 0.0, 0, True],
                                [0.0, 0.75, 4.71238898038469, True],
                                [0.0, 1.75, 4.71238898038469, True]]
                        self.cis = [[-1.5, 1.5, -0.785398163, True],
                                [1.5, -1.5, 2.35619, True]]
                        self.rsu_ip = '127.0.0.1'
                        self.interval = 0.125
                        self.offset_interval = 0.0
                        self.fallthrough_delay = 0.100
                        self.init_time = 10.0 # Seconds to wait for the system to initialize before starting
                        self.map = 0
                        self.map_length = 1.0
                        self.simulation = True
                        self.debug = False
                        self.data_collect_mode = False
                        self.unit_test = False
                        self.unit_test_config = None
                        self.gui_interval = 100 # 10 hz, hopefully this is fast enough
                        self.data_collect_mode = False
                        self.use_global_fusion = True
                        self.cooperative_monitoring = False
                        self.cooperative_monitoring_update = 8 # cycles
                        self.test_one_step_kalman = False
                elif "one_cav_simulation" == setting:
                        print("         Config: one_cav_simulation selected")
                        # Working 4 CAV simulation
                        self.cav = [[-0.75, 0.0, 0, True]]
                        self.cis = []
                        self.rsu_ip = '127.0.0.1'
                        self.interval = 0.125
                        self.offset_interval = 0.0
                        self.fallthrough_delay = 0.100
                        self.init_time = 10.0 # Seconds to wait for the system to initialize before starting
                        self.map = 0
                        self.map_length = 1.0
                        self.simulation = True
                        self.debug = False
                        self.data_collect_mode = False
                        self.unit_test = False
                        self.unit_test_config = None
                        self.gui_interval = 100 # 10 hz, hopefully this is fast enough
                        self.data_collect_mode = False
                        self.use_global_fusion = True
                        self.cooperative_monitoring = False
                        self.cooperative_monitoring_update = 8 # cycles
                        self.test_one_step_kalman = False
                elif "two_cav_simulation_unit_test" == setting:
                        print("         Config: two_cav_simulation_unit_test selected")
                        # Working 4 CAV simulation
                        self.cav = [[-0.75, 0.0, 0, True],
                                [0.0, 0.75, 4.71238898038469, True]]
                        self.cis = []#[[-0.75, 0.75, -0.785398163, True]]#,
                                #[0.75, -0.75, -0.785398163, True]]
                        self.rsu_ip = '127.0.0.1'
                        self.interval = 0.125
                        self.offset_interval = 0.0
                        self.fallthrough_delay = 0.100
                        self.init_time = 10.0 # Seconds to wait for the system to initialize before starting
                        self.map = 0
                        self.map_length = 1.0
                        self.simulation = True
                        self.debug = False
                        self.unit_test = True
                        self.unit_test_speed_target = .4
                        self.unit_test_time = 120.0
                        self.unit_test_config = [[0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], 
                                                [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True]]
                        self.gui_interval = 100 # 10 hz, hopefully this is fast enough
                        self.data_collect_mode = False
                        self.use_global_fusion = True
                        self.cooperative_monitoring = True
                        self.cooperative_monitoring_update = 8 # cycles
                        self.test_one_step_kalman = False
                elif "two_cav_simulation_unit_test_2" == setting:
                        print("         Config: two_cav_simulation_unit_test selected")
                        # Working 4 CAV simulation
                        self.cav = [[-0.75, 0.0, 0, True],
                                [0.0, 0.75, 4.71238898038469, True]]
                        self.cis = []#[[-0.75, 0.75, -0.785398163, True]]#,
                                #[0.75, -0.75, -0.785398163, True]]
                        self.rsu_ip = '127.0.0.1'
                        self.interval = 0.125
                        self.offset_interval = 0.0
                        self.fallthrough_delay = 0.100
                        self.init_time = 10.0 # Seconds to wait for the system to initialize before starting
                        self.map = 0
                        self.map_length = 2.0
                        self.simulation = True
                        self.debug = False
                        self.unit_test = True
                        self.unit_test_speed_target = .4
                        self.unit_test_time = 120.0
                        self.unit_test_config = [[0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], 
                                                [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True]]
                        self.gui_interval = 100 # 10 hz, hopefully this is fast enough
                        self.data_collect_mode = False
                        self.use_global_fusion = True
                        self.cooperative_monitoring = True
                        self.cooperative_monitoring_update = 8 # cycles
                        self.test_one_step_kalman = False
                elif "two_cav_simulation_unit_test_3" == setting:
                        print("         Config: two_cav_simulation_unit_test selected")
                        # Working 4 CAV simulation
                        self.cav = [[-0.75, 0.0, 0, True],
                                [0.0, 0.75, 4.71238898038469, True]]
                        self.cis = [[-0.75, 0.75, -0.785398163, True]]#,
                                #[0.75, -0.75, -0.785398163, True]]
                        self.rsu_ip = '127.0.0.1'
                        self.interval = 0.125
                        self.offset_interval = 0.0
                        self.fallthrough_delay = 0.100
                        self.init_time = 10.0 # Seconds to wait for the system to initialize before starting
                        self.map = 0
                        self.map_length = 1.0
                        self.simulation = True
                        self.debug = False
                        self.unit_test = True
                        self.unit_test_speed_target = .4
                        self.unit_test_time = 120.0
                        self.unit_test_config = [[0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], 
                                                [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True]]
                        self.gui_interval = 100 # 10 hz, hopefully this is fast enough
                        self.data_collect_mode = False
                        self.use_global_fusion = True
                        self.cooperative_monitoring = True
                        self.cooperative_monitoring_update = 8 # cycles
                        self.test_one_step_kalman = False
                elif "two_cav_simulation_unit_test_4" == setting:
                        print("         Config: two_cav_simulation_unit_test selected")
                        # Working 4 CAV simulation
                        self.cav = [[-0.75, 0.0, 0, True],
                                [0.0, 0.75, 4.71238898038469, True]]
                        self.cis = [[-0.75, 0.75, -0.785398163, True]]#,
                                #[0.75, -0.75, -0.785398163, True]]
                        self.rsu_ip = '127.0.0.1'
                        self.interval = 0.125
                        self.offset_interval = 0.0
                        self.fallthrough_delay = 0.100
                        self.init_time = 10.0 # Seconds to wait for the system to initialize before starting
                        self.map = 0
                        self.map_length = 2.0
                        self.simulation = True
                        self.debug = False
                        self.unit_test = True
                        self.unit_test_speed_target = .4
                        self.unit_test_time = 120.0
                        self.unit_test_config = [[0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], 
                                                [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True]]
                        self.gui_interval = 100 # 10 hz, hopefully this is fast enough
                        self.data_collect_mode = False
                        self.use_global_fusion = True
                        self.cooperative_monitoring = True
                        self.cooperative_monitoring_update = 8 # cycles
                        self.test_one_step_kalman = False
                elif "four_cav_simulation_unit_test" == setting:
                        print("         Config: four_cav_simulation_unit_test selected")
                        # Working 4 CAV simulation
                        self.cav = [[-0.75, 0.0, 0, True],
                                [-1.5, 0.0, 0, True],
                                [0.0, 0.75, 4.71238898038469, True],
                                [0.0, 1.75, 4.71238898038469, True]]
                        self.cis = [] #[[-0.75, 0.75, -0.785398163, True]]#,
                                #[0.75, -0.75, -0.785398163, True]]
                        self.rsu_ip = '127.0.0.1'
                        self.interval = 0.125
                        self.offset_interval = 0.0
                        self.fallthrough_delay = 0.100
                        self.init_time = 10.0 # Seconds to wait for the system to initialize before starting
                        self.map = 0
                        self.map_length = 2.0
                        self.simulation = True
                        self.debug = False
                        self.data_collect_mode = False
                        self.unit_test = True
                        self.unit_test_speed_target = .4
                        self.unit_test_time = 120.0
                        self.unit_test_config = [[0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], [0,0,False], 
                                                [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True], [0,0,True]]
                        self.gui_interval = 100 # 10 hz, hopefully this is fast enough
                        self.data_collect_mode = False
                        self.use_global_fusion = True
                        self.cooperative_monitoring = False
                        self.cooperative_monitoring_update = 8 # cycles
                        self.test_one_step_kalman = False
                else:
                        print("         Config: two_cav_simulation selected")
                        # Working 2 CAV simulation
                        self.cav = [[-0.75, 0.0, 0, True],
                                [0.0, 0.75, 4.71238898038469, True]]
                        self.cis = []#[[-1.5, 1.5, -0.785398163, True]]
                        self.rsu_ip = '127.0.0.1'
                        self.interval = 0.125
                        self.offset_interval = 0.0
                        self.fallthrough_delay = 0.100
                        self.init_time = 10.0 # Seconds to wait for the system to initialize before starting
                        self.map = 0
                        self.map_length = 2.0
                        self.simulation = True
                        self.debug = False
                        self.unit_test = False
                        self.unit_test_config = None
                        self.gui_interval = 100 # 10 hz, hopefully this is fast enough
                        self.data_collect_mode = False
                        self.use_global_fusion = True
                        self.cooperative_monitoring = False
                        self.cooperative_monitoring_update = 8 # cycles
                        self.test_one_step_kalman = False
                print(" Config complete, using map: ", self.map)