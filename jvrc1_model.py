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
        self.registerNamedPose('sim_half_sit',
                               [ -7.89611370e-01, 0,  0,  1.45842498e+00, 0, -0.66881361,
                                 -7.89611370e-01, 0,  0,  1.45842498e+00, 0, -0.66881361,
                                 0.,  0.13,  0.,    0., 0.,   0.,
                                 -0.052, -0.17,  0.,   -0.52, 0.,   0.,    0.,    0.,   0.,    0.,    0., 0.,   0.,
                                 -0.052,  0.17,  0.,   -0.52, 0.,   0.,    0.,    0.,   0.,    0.,    0., 0.,   0., ],
                               ru.make_coordinates( {'pos': [-0.072161, 0.0, 0.702 ]} )
                               )

    def setSimDefaultPose(self):
        self.setNamedPose('sim_default')
    def setHalfSitPose(self):
        self.setNamedPose('sim_half_sit')
    def getJointSettings(self, Pratio=1.0, Dratio=1.0):
        settings = {
            'R_HIP_P':     {'P': 5000 *Pratio, 'D': 30*Dratio },
            'R_HIP_R':     {'P': 5000 *Pratio, 'D': 30*Dratio },
            'R_HIP_Y':     {'P': 5000 *Pratio, 'D': 30*Dratio },
            'R_KNEE':      {'P': 6000 *Pratio, 'D': 50*Dratio },
            'R_ANKLE_R':   {'P': 3000 *Pratio, 'D': 20*Dratio },
            'R_ANKLE_P':   {'P': 3000 *Pratio, 'D': 20*Dratio },
            'L_HIP_P':     {'P': 5000 *Pratio, 'D': 30*Dratio },
            'L_HIP_R':     {'P': 5000 *Pratio, 'D': 30*Dratio },
            'L_HIP_Y':     {'P': 5000 *Pratio, 'D': 30*Dratio },
            'L_KNEE':      {'P': 6000 *Pratio, 'D': 50*Dratio },
            'L_ANKLE_R':   {'P': 3000 *Pratio, 'D': 20*Dratio },
            'L_ANKLE_P':   {'P': 3000 *Pratio, 'D': 20*Dratio },
            'WAIST_Y':     {'P': 5000 *Pratio, 'D': 32*Dratio },
            'WAIST_P':     {'P': 5000 *Pratio, 'D': 32*Dratio },
            'WAIST_R':     {'P': 5000 *Pratio, 'D': 32*Dratio },
            'NECK_Y':      {'P': 1000 *Pratio, 'D': 21*Dratio },
            'NECK_R':      {'P': 1000 *Pratio, 'D': 21*Dratio },
            'NECK_P':      {'P': 1000 *Pratio, 'D': 21*Dratio },
            'R_SHOULDER_P':{'P': 1100 *Pratio, 'D': 70*Dratio },
            'R_SHOULDER_R':{'P': 1100 *Pratio, 'D': 70*Dratio },
            'R_SHOULDER_Y':{'P': 1100 *Pratio, 'D': 70*Dratio },
            'R_ELBOW_P':   {'P': 1200 *Pratio, 'D': 80*Dratio },
            'R_ELBOW_Y':   {'P': 1200 *Pratio, 'D': 80*Dratio },
            'R_WRIST_R':   {'P': 500  *Pratio, 'D': 24*Dratio },
            'R_WRIST_Y':   {'P': 500  *Pratio, 'D': 24*Dratio },
            'R_UTHUMB':    {'P': 30   *Pratio, 'D': 3 *Dratio },
            'R_LTHUMB':    {'P': 30   *Pratio, 'D': 3 *Dratio },
            'R_UINDEX':    {'P': 30   *Pratio, 'D': 3 *Dratio },
            'R_LINDEX':    {'P': 30   *Pratio, 'D': 3 *Dratio },
            'R_ULITTLE':   {'P': 30   *Pratio, 'D': 3 *Dratio },
            'R_LLITTLE':   {'P': 30   *Pratio, 'D': 3 *Dratio },
            'L_SHOULDER_P':{'P': 1100 *Pratio, 'D': 70*Dratio },
            'L_SHOULDER_R':{'P': 1100 *Pratio, 'D': 70*Dratio },
            'L_SHOULDER_Y':{'P': 1100 *Pratio, 'D': 70*Dratio },
            'L_ELBOW_P':   {'P': 1200 *Pratio, 'D': 80*Dratio },
            'L_ELBOW_Y':   {'P': 1200 *Pratio, 'D': 80*Dratio },
            'L_WRIST_R':   {'P': 500  *Pratio, 'D': 24*Dratio },
            'L_WRIST_Y':   {'P': 500  *Pratio, 'D': 24*Dratio },
            'L_UTHUMB':    {'P': 30   *Pratio, 'D': 3 *Dratio },
            'L_LTHUMB':    {'P': 30   *Pratio, 'D': 3 *Dratio },
            'L_UINDEX':    {'P': 30   *Pratio, 'D': 3 *Dratio },
            'L_LINDEX':    {'P': 30   *Pratio, 'D': 3 *Dratio },
            'L_ULITTLE':   {'P': 30   *Pratio, 'D': 3 *Dratio },
            'L_LLITTLE':   {'P': 30   *Pratio, 'D': 3 *Dratio },
        }
        return settings

### settings of model_file
JVRC1.model_file = f'{os.path.dirname(__file__)}/JVRC-1/main.wrl'

### robot_class: 
robot_class = JVRC1

### makeRobot(robot=None, **kwargs):
def makeRobot(robot=None, item=True, world=True, **kwargs):
    return robot_class(robot, item=item, world=world, **kwargs)
