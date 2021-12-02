import math
import pprint
import time
from pick_and_place_arm_sm import PickAndPlaceStateMachine, PickAndPlaceState, PSM_HOME_POS, vector_eps_eq


PSM_HOME_POS = PyKDL.Vector(0., 0., -0.1)

class PickAndPlaceDualArmStateMachine:
    # this class isn't actually a state machine, but rather a way to connect 
    # the states of PSM1 and PSM2 state machines to effectively create a 
    # 2-arm FSM

    def _get_objects_for_psms(self):
        '''
        Returns a dict of PSM index -> list of objects that are closest to that PSM
        '''
        result = dict()
        for object in self.world.objects:
            closest_psm_idx = self.world_to_psm_tfs.index(
                min(self.world_to_psm_tfs, key=lambda tf : (tf * object.pos).Norm()))
            
            if closest_psm_idx not in result:
                result[closest_psm_idx] = list()
            
            result[closest_psm_idx].append(object)

        if self.log_verbose:
            loginfo("Unpicked objects: {}".format(pprint.pformat(result)))

        return result

    def __init__(self, psms, world_to_psm_tfs, world, approach_vec, log_verbose=False):
        self.world = world
        self.psms = psms
        self.world_to_psm_tfs = world_to_psm_tfs
        self.log_verbose = log_verbose
        self.approach_vec = approach_vec
        # get objects for psms
        psm_to_objects_map = self._get_objects_for_psms()
        print(psm_to_objects_map)
        if 0 in psm_to_objects_map:
            self.current_sm = PickAndPlaceStateMachine(self.psms[0], self.world,
                                                    self.world_to_psm_tfs[0], psm_to_objects_map[0][0],
                                                    approach_vec, closed_loop=False, 
                                                    home_when_done=bool(len(psm_to_objects_map[0]) <= 1))
        elif 1 in psm_to_objects_map:
            self.current_sm = PickAndPlaceStateMachine(self.psms[1], self.world,
                                                    self.world_to_psm_tfs[1], psm_to_objects_map[1][0],
                                                    approach_vec, closed_loop=False)

    def update_world(self, world):
        self.world = world
        self.current_sm.update_world(world)


    def run_once(self):
        if self.current_sm.is_done() and \
            self.current_sm.psm.get_current_jaw_position() > math.pi / 3:
            objects = self._get_objects_for_psms()
            if 0 in objects:
                print(len(objects[0]))
                self.current_sm = \
                    PickAndPlaceStateMachine(self.psms[0], self.world,
                                            self.world_to_psm_tfs[0], objects[0][0],
                                            self.approach_vec, closed_loop=False, home_when_done=False)
            elif 1 in objects:
                if self.current_sm.psm == self.psms[0]:
                    # go home
                    self.current_sm.state = PickAndPlaceState.HOME
                    if not self.current_sm.is_done():
                        self.current_sm.run_once()

                    # this state check is here because i want to sleep
                    while not vector_eps_eq(self.psms[0].get_current_position().p, PSM_HOME_POS):
                        # spaghetti code
                        time.sleep(0.1)

                self.current_sm = \
                    PickAndPlaceStateMachine(self.psms[1], self.world,
                                            self.world_to_psm_tfs[1], objects[1][0],
                                            self.approach_vec, closed_loop=False, home_when_done=False)

        if not self.current_sm.is_done():
            self.current_sm.run_once()


    def is_done(self):
        return bool(len(self.world.objects) == 0) and self.current_sm.jaw_fully_open()


    def __str__(self):
        return str(self.__dict__)

    def __repr__(self):
        return str(self.__dict__)