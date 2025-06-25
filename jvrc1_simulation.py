from . import jvrc1_model as rmodel
from irsl_choreonoid.simulation_utils import setupSimEnv
from irsl_choreonoid.simulation_utils import startSimEnv

def simSetup(addFloor=True, initialCoords=None):
    """
    Setup simulation environment

    Args:
        addFloor (boolean, default=True) :
        initialCoords (coordinates, optional) :

    Returns:
        (simenv, robot) :

    Examples:
        >>>> simenv, robot = simSetup()
        >>>> simStart(simenv, robot)
        >>>> simenv.run(10.0)
        >>>> simenv.stop()
    """
    return setupSimEnv(rmodel.robot_class, addFloor, initialCoords)

def simStart(simenv, mrobot, pre_wait=2.0):
    """
    Start simulation

    Args:
        simenv () : Results of simSetup
        mrobot () : Results of simSetup

    Returns:
        None

    Examples:
        >>>> simenv, robot = simSetup()
        >>>> simStart(simenv, robot)
        >>>> simenv.run(10.0)
        >>>> simenv.stop()
    """
    startSimEnv(simenv, mrobot, pre_wait=pre_wait)
