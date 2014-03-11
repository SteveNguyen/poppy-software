
from scenography import *
from poppytools.configuration.config import poppy_config
import pypot.robot
import time
poppy = pypot.robot.from_config(poppy_config)
poppy.start_sync()

time.sleep(2)
init_sceno = InitScenography(poppy)
leapC = LeapMotionCuriosity(poppy)

stand=StandPosition(poppy)
sit=SitPosition(poppy)

init_sceno.start()
init_sceno.wait_to_stop()

stand.start()
stand.wait_to_stop()

sit.start()

leapC.start()


time.sleep(100)
