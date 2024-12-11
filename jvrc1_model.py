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
        len_a = 0.08
        len_b = 0.08
        width = 0.1
        theta = 45/180.0*PI
        toe = coordinates(fv(0, 0, self.off_))
        toe.translate(fv(len_a, 0, 0))
        toe.rotate(-theta, coordinates.Y)
        toe.translate(fv(0.5*len_b, 0, 0))
        self.registerEndEffector('llegt', ## end-effector
                                 'L_ANKLE_P', ## tip-link
                                 tip_link_to_eef = toe,
                                 joint_tuples = (('L_HIP_P', 'hip-p'),
                                                 ('L_HIP_R', 'hip-r'),
                                                 ('L_HIP_Y', 'hip-y'),
                                                 ('L_KNEE', 'knee-p'),
                                                 ('L_ANKLE_R', 'ankle-r'),
                                                 ('L_ANKLE_P', 'ankle-p'),
                                                 )
                                 )
        self.registerEndEffector('rlegt', ## end-effector
                                 'R_ANKLE_P', ## tip-link
                                 tip_link_to_eef = toe.copy(),
                                 joint_tuples = (('R_HIP_P', 'hip-p'),
                                                 ('R_HIP_R', 'hip-r'),
                                                 ('R_HIP_Y', 'hip-y'),
                                                 ('R_KNEE', 'knee-p'),
                                                 ('R_ANKLE_R', 'ankle-r'),
                                                 ('R_ANKLE_P', 'ankle-p'),
                                                 )
                                 )
        self.foot2toe = coordinates(fv(len_a + 0.5*len_b*math.cos(theta), 0, 0))
        ## llegt.inverseKinematics( lleg.endEffector.transform(robot.foot2toe) ) ###
        self._makefoot(len_a, len_b, len_a, theta, width=width)##
        #### old points
        #self.setFrame('rf_point0', 'R_ANKLE_P', coordinates(fv( 0.157,  0.05, self.off_)))
        #self.setFrame('rf_point1', 'R_ANKLE_P', coordinates(fv( 0.157, -0.05, self.off_)))
        #self.setFrame('rf_point2', 'R_ANKLE_P', coordinates(fv(-0.095,  0.05, self.off_)))
        #self.setFrame('rf_point3', 'R_ANKLE_P', coordinates(fv(-0.095, -0.05, self.off_)))
        #self.setFrame('lf_point0', 'L_ANKLE_P', coordinates(fv( 0.157,  0.05, self.off_)))
        #self.setFrame('lf_point1', 'L_ANKLE_P', coordinates(fv( 0.157, -0.05, self.off_)))
        #self.setFrame('lf_point2', 'L_ANKLE_P', coordinates(fv(-0.095,  0.05, self.off_)))
        #self.setFrame('lf_point3', 'L_ANKLE_P', coordinates(fv(-0.095, -0.05, self.off_)))
        #### new points
        self.setFrame('rf_point0', 'R_ANKLE_P', coordinates(fv( len_a,  0.5*width, self.off_)))
        self.setFrame('rf_point1', 'R_ANKLE_P', coordinates(fv( len_a, -0.5*width, self.off_)))
        self.setFrame('rf_point2', 'R_ANKLE_P', coordinates(fv(-len_a,  0.5*width, self.off_)))
        self.setFrame('rf_point3', 'R_ANKLE_P', coordinates(fv(-len_a, -0.5*width, self.off_)))
        #
        self.setFrame('lf_point0', 'L_ANKLE_P', coordinates(fv( len_a,  0.5*width, self.off_)))
        self.setFrame('lf_point1', 'L_ANKLE_P', coordinates(fv( len_a, -0.5*width, self.off_)))
        self.setFrame('lf_point2', 'L_ANKLE_P', coordinates(fv(-len_a,  0.5*width, self.off_)))
        self.setFrame('lf_point3', 'L_ANKLE_P', coordinates(fv(-len_a, -0.5*width, self.off_)))
        #### new points
        rr=coordinates(fv( len_a,  0.5*width, self.off_), fv(0, -theta, 0))
        rl=coordinates(fv( len_a, -0.5*width, self.off_), fv(0, -theta, 0))
        self.setFrame('rtoe_point0', 'R_ANKLE_P', rr)
        self.setFrame('rtoe_point1', 'R_ANKLE_P', rl)
        self.setFrame('rtoe_point2', 'R_ANKLE_P', rr.translate(fv(len_b, 0, 0)))
        self.setFrame('rtoe_point3', 'R_ANKLE_P', rl.translate(fv(len_b, 0, 0)))
        lr=coordinates(fv( len_a,  0.5*width, self.off_), fv(0, -theta, 0))
        ll=coordinates(fv( len_a, -0.5*width, self.off_), fv(0, -theta, 0))
        self.setFrame('ltoe_point0', 'L_ANKLE_P', lr)
        self.setFrame('ltoe_point1', 'L_ANKLE_P', ll)
        self.setFrame('ltoe_point2', 'L_ANKLE_P', lr.translate(fv(len_b, 0, 0)))
        self.setFrame('ltoe_point3', 'L_ANKLE_P', ll.translate(fv(len_b, 0, 0)))

    def _makefoot(self, length_a, length_b, length_c, theta, width, ti=0.02):
        sth = math.sin(theta)
        cth = math.cos(theta)
        cros = [[-length_c, 0],
                [length_a, 0],
                [length_a + length_b*cth,          length_b*sth],
                [length_a + length_b*cth - ti*sth, length_b*sth + ti*cth],
                [length_a - ti*sth/cth, ti],
                [-length_c, ti],
                [-length_c, 0]]
        obj = mkshapes.makeExtrusion(cros, [[0, -0.5*width, 0], [0, 0.5*width, 0]], rawShape=True, color=[1.0*0.5, 0.65*0.5, 0])
        trs = cutil.SgPosTransform()
        trs.T = coordinates(fv(0, 0, self.off_)).cnoidPosition
        trs.addChild(obj)
        ##
        jtr = self.robot.joint('R_ANKLE_P')
        jtr.clearShapeNodes()
        jtr.addShapeNode(trs)
        jtl = self.robot.joint('L_ANKLE_P')
        jtl.clearShapeNodes()
        jtl.addShapeNode(trs)

    @property
    def llegt(self):
        return self.getLimb('llegt')

    @property
    def rlegt(self):
        return self.getLimb('rlegt')

### settings of model_file
JVRC1.model_file = f'{os.path.dirname(__file__)}/JVRC-1/main.wrl'

### robot_class: 
robot_class = JVRC1

### makeRobot(robot=None, **kwargs):
def makeRobot(robot=None, item=True, world=True, **kwargs):
    return robot_class(robot, item=item, world=world, **kwargs)
