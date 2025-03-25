# jupyter console --kernel=choreonoid

exec(open('/choreonoid_ws/install/share/irsl_choreonoid/sample/irsl_import.py').read())
import humanoid_research.robot_graph.make_robot_graph as mrg


lst=[
    {'node': 'PELVIS', 'type': 'link',
     #'mass-param': {'mass': 10.0, 'COM': [-0.01, 0.0, 0.034], 'inertia': [[0.08958333333333333, 0.0, 0.0], [0.0, 0.08958333333333333, 0.0], [0.0, 0.0, 0.11249999999999999]]},
     'mass-param': { 'mass': 41.400000000000006, 'COM': [4.60620498e-02, 4.69297172e-18, 2.50799148e-01],
                     'inertia': [[4.12257542e+00, 1.82145965e-17, 4.49145467e-02], [1.82145965e-17, 2.75815654e+00, 4.85722573e-17],  [4.49145467e-02, 4.85722573e-17, 1.83823196e+00]] },
     'joint': {'type': 'root'}, 'translation': [0.0, 0.0, 0.854], 'rotation': [1.0, 0.0, 0.0, 0.0]},
    ## geom
    {'node': 'G_Body', 'type': 'geom', 'geometry': {'primitive': 'box', 'args': {'x': 0.18, 'y': 0.36, 'z': 0.54, 'color': [0, 0.4, 0]}},
     'relative': True, 'translation': [0.046, 0.0,  0.27 + 0.08], 'rotation': [1.0, 0.0, 0.0, 0.0]},
    [
        {'node': 'R_HIP_P', 'type': 'link', 'mass-param': {'mass': 1.0, 'COM': [0.0, 0.0, 0.0], 'inertia': [[0.0019600000000000004, 0.0, 0.0], [0.0, 0.0019600000000000004, 0.0], [0.0, 0.0, 0.0019600000000000004]]}, 'joint': {'type': 'revolute', 'axis': [0.0, 1.0, 0.0], 'id': 0},
         'translation': [0.0, -0.096, 0.854], 'rotation': [1.0, 0.0, 0.0, 0.0]},
        {'node': 'R_HIP_R', 'type': 'link', 'mass-param': {'mass': 1.0, 'COM': [0.0, 0.0, 0.0], 'inertia': [[0.0019600000000000004, 0.0, 0.0], [0.0, 0.0019600000000000004, 0.0], [0.0, 0.0, 0.0019600000000000004]]}, 'joint': {'type': 'revolute', 'axis': [1.0, 0.0, 0.0], 'id': 1}, 'translation': [0.0, -0.096, 0.854], 'rotation': [1.0, 0.0, 0.0, 0.0]},
        {'node': 'R_HIP_Y', 'type': 'link', 'mass-param': {'mass': 3.0, 'COM': [0.01, 0.0, -0.22], 'inertia': [[0.03192500000000001, 0.0, 0.0], [0.0, 0.03452500000000001, 0.0], [0.0, 0.0, 0.00865]]}, 'joint': {'type': 'revolute', 'axis': [0.0, 0.0, 1.0], 'id': 2}, 'translation': [0.0, -0.096, 0.854], 'rotation': [1.0, 0.0, 0.0, 0.0]},
        {'node': 'R_KNEE', 'type': 'link', 'mass-param': {'mass': 3.0, 'COM': [0.04, 0.0, -0.16], 'inertia': [[0.03192500000000001, 0.0, 0.0], [0.0, 0.03452500000000001, 0.0], [0.0, 0.0, 0.00865]]}, 'joint': {'type': 'revolute', 'axis': [0.0, 1.0, 0.0], 'id': 3}, 'translation': [-0.02, -0.096, 0.46499999999999997], 'rotation': [1.0, 0.0, 0.0, 0.0]},
        {'node': 'R_ANKLE_R', 'type': 'link', 'mass-param': {'mass': 1.0, 'COM': [0.0, 0.0, 0.0], 'inertia': [[0.00064, 0.0, 0.0], [0.0, 0.00064, 0.0], [0.0, 0.0, 0.00064]]}, 'joint': {'type': 'revolute', 'axis': [1.0, 0.0, 0.0], 'id': 4}, 'translation': [0.02, -0.096, 0.10799999999999998], 'rotation': [1.0, 0.0, 0.0, 0.0]},
        {'node': 'R_ANKLE_P', 'type': 'link', 'mass-param': {'mass': 1.5, 'COM': [0.03, 0.0, -0.07], 'inertia': [[0.0014166666666666668, 0.0, 0.0], [0.0, 0.0056166666666666665, 0.0], [0.0, 0.0, 0.006216666666666666]]}, 'joint': {'type': 'revolute', 'axis': [0.0, 1.0, 0.0], 'id': 5}, 'translation': [0.02, -0.096, 0.10799999999999998], 'rotation': [1.0, 0.0, 0.0, 0.0]},
        ## geom
        {'node': 'R_foot', 'type': 'geom', 'geometry': {'primitive': 'box', 'args': {'x': 0.24, 'y': 0.10, 'z': 0.02, 'color': [0, 0.4, 0]}},
         'relative': True, 'translation': [0.025, 0.0,  0.01 - 0.10799999999999998], 'rotation': [1.0, 0.0, 0.0, 0.0]},
    ],
    [
        {'node': 'L_HIP_P', 'type': 'link', 'mass-param': {'mass': 1.0, 'COM': [0.0, 0.0, 0.0], 'inertia': [[0.0019600000000000004, 0.0, 0.0], [0.0, 0.0019600000000000004, 0.0], [0.0, 0.0, 0.0019600000000000004]]}, 'joint': {'type': 'revolute', 'axis': [0.0, 1.0, 0.0], 'id': 6}, 'translation': [0.0, 0.096, 0.854], 'rotation': [1.0, 0.0, 0.0, 0.0]},
        {'node': 'L_HIP_R', 'type': 'link', 'mass-param': {'mass': 1.0, 'COM': [0.0, 0.0, 0.0], 'inertia': [[0.0019600000000000004, 0.0, 0.0], [0.0, 0.0019600000000000004, 0.0], [0.0, 0.0, 0.0019600000000000004]]}, 'joint': {'type': 'revolute', 'axis': [1.0, 0.0, 0.0], 'id': 7}, 'translation': [0.0, 0.096, 0.854], 'rotation': [1.0, 0.0, 0.0, 0.0]},
        {'node': 'L_HIP_Y', 'type': 'link', 'mass-param': {'mass': 3.0, 'COM': [0.01, 0.0, -0.22], 'inertia': [[0.03192500000000001, 0.0, 0.0], [0.0, 0.03452500000000001, 0.0], [0.0, 0.0, 0.00865]]}, 'joint': {'type': 'revolute', 'axis': [0.0, 0.0, 1.0], 'id': 8}, 'translation': [0.0, 0.096, 0.854], 'rotation': [1.0, 0.0, 0.0, 0.0]},
        {'node': 'L_KNEE', 'type': 'link', 'mass-param': {'mass': 3.0, 'COM': [0.04, 0.0, -0.16], 'inertia': [[0.03192500000000001, 0.0, 0.0], [0.0, 0.03452500000000001, 0.0], [0.0, 0.0, 0.00865]]}, 'joint': {'type': 'revolute', 'axis': [0.0, 1.0, 0.0], 'id': 9}, 'translation': [-0.02, 0.096, 0.46499999999999997], 'rotation': [1.0, 0.0, 0.0, 0.0]},
        {'node': 'L_ANKLE_R', 'type': 'link', 'mass-param': {'mass': 1.0, 'COM': [0.0, 0.0, 0.0], 'inertia': [[0.00064, 0.0, 0.0], [0.0, 0.00064, 0.0], [0.0, 0.0, 0.00064]]}, 'joint': {'type': 'revolute', 'axis': [1.0, 0.0, 0.0], 'id': 10}, 'translation': [0.02, 0.096, 0.10799999999999998], 'rotation': [1.0, 0.0, 0.0, 0.0]},
        {'node': 'L_ANKLE_P', 'type': 'link', 'mass-param': {'mass': 1.5, 'COM': [0.03, 0.0, -0.07], 'inertia': [[0.0014166666666666668, 0.0, 0.0], [0.0, 0.0056166666666666665, 0.0], [0.0, 0.0, 0.006216666666666666]]}, 'joint': {'type': 'revolute', 'axis': [0.0, 1.0, 0.0], 'id': 11}, 'translation': [0.02, 0.096, 0.10799999999999998], 'rotation': [1.0, 0.0, 0.0, 0.0]},
        ## geom
        {'node': 'L_foot', 'type': 'geom', 'geometry': {'primitive': 'box', 'args': {'x': 0.24, 'y': 0.10, 'z': 0.02, 'color': [0, 0.4, 0]}},
         'relative': True, 'translation': [0.025, 0.0,  0.01 - 0.10799999999999998], 'rotation': [1.0, 0.0, 0.0, 0.0]},
    ],
]

gg = mrg.RobotTree.generate_from_list(lst, add_root=True)
gg.update_coords()
#
gg.add_geometries_for_joints(scale=0.2)
gg.add_geometries_for_links(scale=0.2)

rtb = mrg.RobotTreeBuilder()
rtb.buildRobotFromTree(gg)
for j in rtb.body.joints:
    if j.name == j.jointName:
        j.setName('{}_S'.format(j.jointName))
    j.setJointEffortRange(-400, 400)
    j.setJointVelocityRange(-18, 18)
    j.setJointRange(-3.2, 3.2)

rtb.viewInfo()
rtb.exportBody('simplej.body')
#######

#### check upper-bodie's mass-parameter
import jvrc1_model.jvrc1_model as rmodel
robot=rmodel.robot_class()
robot.setSimDefaultPose()

all_lks = robot.linkList
leg_lks = robot.jointList[0:12]
upper_lks =  list( set(all_lks) - set(leg_lks) )
idx = upper_lks.index( robot.robot.rootLink )
upper_lks[0], upper_lks[idx] = upper_lks[idx], upper_lks[0]
ru.mergedMassPropertyOfList(upper_lks)


#### removed upperbody
itm=ru.loadRobotItem('jvrc1_model/JVRC-1/main.wrl', world=False)
itm.body.rootLink.removeChild( itm.body.link('WAIST_Y') )
itm.body.updateLinkTree()

gg = mrg.RobotTree.generate_from_body(itm.body)
gg.update_coords()
lst=gg.make_list()

print(lst[0])
for l in lst[1]:
    print(l)
for l in lst[2:]:
    print(l)
