from . import jvrc1_model as rmodel
from irsl_choreonoid.simulation_utils import setupSimEnv
from irsl_choreonoid.simulation_utils import startSimEnv

def simSetup(addFloor=True, initialCoords=None, initialPose=None):
    """
    Setup simulation environment

    Args:
        addFloor (boolean, default=True) :
        initialCoords (coordinates, optional) :
        initialPose (str or np.array or boolean, optional) :

    Returns:
        (simenv, robot) :

    Examples:
        >>>> simenv, robot = simSetup()
        >>>> simStart(simenv, robot)
        >>>> simenv.run(10.0)
        >>>> simenv.stop()
    """
    return setupSimEnv(rmodel.robot_class, addFloor=addFloor,
                       initialCoords=initialCoords, initialPose=initialPose)

def simStart(simenv, mrobot, pre_wait=2.0, Pratio=1.0, Dratio=1.0):
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
    startSimEnv(simenv, mrobot, pre_wait=pre_wait, Pratio=Pratio, Dratio=Dratio)
