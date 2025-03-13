import os
exec(open('/choreonoid_ws/install/share/irsl_choreonoid/sample/irsl_import.py').read())

class JVRC1(ru.ImportedRobotModel):
    def __init__(self, robot=None, item=True, world=False, **kwargs):
        super().__init__(robot=robot, item=item, world=world, **kwargs)

    def _init_ending(self, **kwargs): ## override
        self.off_ = (-0.02 + -0.08773589462041854) ## offset from ankle_joint to bottom of foot-plate
        self.registerEndEffector('lleg', ## end-effector
                                 'L_ANKLE_P', ## tip-link
                                 tip_link_to_eef = coordinates(fv(0, 0, self.off_)),
                                 joint_tuples = (('L_HIP_P', 'hip-p'),
                                                 ('L_HIP_R', 'hip-r'),
                                                 ('L_HIP_Y', 'hip-y'),
                                                 ('L_KNEE', 'knee-p'),
                                                 ('L_ANKLE_R', 'ankle-r'),
                                                 ('L_ANKLE_P', 'ankle-p'),
                                                 )
                                 )
        self.registerEndEffector('rleg', ## end-effector
                                 'R_ANKLE_P', ## tip-link
                                 tip_link_to_eef = coordinates(fv(0, 0, self.off_)),
                                 joint_tuples = (('R_HIP_P', 'hip-p'),
                                                 ('R_HIP_R', 'hip-r'),
                                                 ('R_HIP_Y', 'hip-y'),
                                                 ('R_KNEE', 'knee-p'),
                                                 ('R_ANKLE_R', 'ankle-r'),
                                                 ('R_ANKLE_P', 'ankle-p'),
                                                 )
                                 )
        ##
        self.setFrame('rf_point0', 'R_ANKLE_P', coordinates(fv( 0.157,  0.05, self.off_)))
        self.setFrame('rf_point1', 'R_ANKLE_P', coordinates(fv( 0.157, -0.05, self.off_)))
        self.setFrame('rf_point2', 'R_ANKLE_P', coordinates(fv(-0.095,  0.05, self.off_)))
        self.setFrame('rf_point3', 'R_ANKLE_P', coordinates(fv(-0.095, -0.05, self.off_)))
        ##
        self.setFrame('lf_point0', 'L_ANKLE_P', coordinates(fv( 0.157,  0.05, self.off_)))
        self.setFrame('lf_point1', 'L_ANKLE_P', coordinates(fv( 0.157, -0.05, self.off_)))
        self.setFrame('lf_point2', 'L_ANKLE_P', coordinates(fv(-0.095,  0.05, self.off_)))
        self.setFrame('lf_point3', 'L_ANKLE_P', coordinates(fv(-0.095, -0.05, self.off_)))
        #
        self.registerNamedPose('sim_default', ## CoM = 0, 0, xxx
                               [ -0.38, -0.01, 0., 0.72, -0.01, -0.33,
                                 -0.38,  0.02, 0., 0.72, -0.02, -0.33,
                                 0.,  0.13,  0.,    0., 0.,   0.,
                                 -0.052, -0.17,  0.,   -0.52, 0.,   0.,    0.,    0.,   0.,    0.,    0., 0.,   0.,
                                 -0.052,  0.17,  0.,   -0.52, 0.,   0.,    0.,    0.,   0.,    0.,    0., 0.,   0., ],
                               ru.make_coordinates( {'pos': [0.0, 0.0, 0.825 ]} )
                               )
    def setSimDefaultPose(self):
        self.setNamedPose('sim_default')

### settings of model_file
JVRC1.model_file = f'{os.path.dirname(__file__)}/JVRC-1/main.wrl'

### robot_class: 
robot_class = JVRC1

### makeRobot(robot=None, **kwargs):
def makeRobot(robot=None, item=True, world=True, **kwargs):
    return robot_class(robot, item=item, world=world, **kwargs)
