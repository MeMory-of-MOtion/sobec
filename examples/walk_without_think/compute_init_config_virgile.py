import pinocchio as pin
import numpy as np
import matplotlib.pylab as plt  # noqa: F401
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401

np.set_printoptions(precision=6, linewidth=350, suppress=True,threshold=1e6)

urdffile= "robot.urdf"
urdfpath = "model_robot_virgile/model_simplified"
urdf = pin.RobotWrapper.BuildFromURDF(urdfpath + "/" + urdffile,urdfpath,
                                      root_joint=pin.JointModelFreeFlyer())
urdf.q0 = pin.neutral(urdf.model)
urdf.q0[2] = +0.5507357853479324 ## So that the feet are at z=0
urdf.model.referenceConfigurations['half_sitting'] = urdf.q0.copy()

model=urdf.model
data=model.createData()
q0 = urdf.q0.copy()


import meshcat
from pinocchio.visualize import MeshcatVisualizer
viz = MeshcatVisualizer(urdf.model, urdf.collision_model, urdf.visual_model)
server = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")
# server = None
viz.initViewer(loadModel=True, viewer=server)

pin.centerOfMass(model,data,q0)
print('Diff COM of ',data.com[0][:2])
q0[:2] -= data.com[0][:2]

pin.framesForwardKinematics(model,data,q0)
pin.centerOfMass(model,data,q0)

print(f'\n-----\nCOM ... \t',data.com[0])
root_id = i=model.getFrameId('root_joint')
print(f'\n-----\n{model.frames[i].name} ... ({model.names[model.frames[i].parent]})',
      data.oMf[i].translation)
rhip_id = i=model.getFrameId('right_hip_z')
print(f'\n-----\n{model.frames[i].name} ... ({model.names[model.frames[i].parent]})',
      data.oMf[i].translation)
rf_id = i=model.getFrameId('right_foot_frame')
print(f'\n-----\n{model.frames[i].name} ... ({model.names[model.frames[i].parent]})',
      data.oMf[i].translation)
lhip_id = i=model.getFrameId('left_hip_z')
print(f'\n-----\n{model.frames[i].name} ... ({model.names[model.frames[i].parent]})',
      data.oMf[i].translation)
lf_id = i=model.getFrameId('left_foot_frame')
print(f'\n-----\n{model.frames[i].name} ... ({model.names[model.frames[i].parent]})',
      data.oMf[i].translation)


# Set target values for left and right feet
data.oMf[rf_id].translation[2]=0
data.oMf[lf_id].translation[2]=0

import casadi
import pinocchio.casadi as cpin

cmodel = cpin.Model(model)
cdata = cmodel.createData()

cq = casadi.SX.sym('q',model.nq)
com = casadi.Function('com',[cq],[cpin.centerOfMass(cmodel,cdata,cq)])
cpin.framesForwardKinematics(cmodel,cdata,cq)
rf =  casadi.Function('rf',[cq],[cdata.oMf[rf_id].translation])
lf =  casadi.Function('lf',[cq],[cdata.oMf[lf_id].translation])
drf =  casadi.Function('drf',[cq],[  cpin.log6(cdata.oMf[rf_id].inverse()*cpin.SE3(data.oMf[rf_id])).vector ])
dlf =  casadi.Function('dlf',[cq],[  cpin.log6(cdata.oMf[lf_id].inverse()*cpin.SE3(data.oMf[lf_id])).vector ])

opti = casadi.Opti()
var_q = opti.variable(model.nq)
# cost = casadi.sumsqr(com(var_q))
# cost += casadi.sumsqr( drf(var_q) )
# cost += casadi.sumsqr( dlf(var_q) )

#opti.subject_to(com(var_q)[:2]==0)
opti.subject_to(com(var_q)[:2]-(rf(var_q)[:2]+lf(var_q)[:2])/2==0)
opti.subject_to(drf(var_q)==0)
opti.subject_to(dlf(var_q)==0)
opti.subject_to(var_q[3:7] == [0,0,0,1] )
cost = casadi.sumsqr( var_q-q0 )

opti.minimize(cost)
opti.solver("ipopt")
opti.solve()
sol_q = opti.value(var_q)


viz.display(sol_q)
pin.framesForwardKinematics(model,data,sol_q)
pin.centerOfMass(model,data,sol_q)
print(f'\n-----\nCOM ... \t',data.com[0])
root_id = i=model.getFrameId('root_joint')
print(f'\n-----\n{model.frames[i].name} ... ({model.names[model.frames[i].parentJoint]})',
      data.oMf[i].translation)
rhip_id = i=model.getFrameId('right_hip_z')
print(f'\n-----\n{model.frames[i].name} ... ({model.names[model.frames[i].parentJoint]})',
      data.oMf[i].translation)
rf_id = i=model.getFrameId('right_foot_frame')
print(f'\n-----\n{model.frames[i].name} ... ({model.names[model.frames[i].parentJoint]})',
      data.oMf[i].translation)
lhip_id = i=model.getFrameId('left_hip_z')
print(f'\n-----\n{model.frames[i].name} ... ({model.names[model.frames[i].parentJoint]})',
      data.oMf[i].translation)
lf_id = i=model.getFrameId('left_foot_frame')
print(f'\n-----\n{model.frames[i].name} ... ({model.names[model.frames[i].parentJoint]})',
      data.oMf[i].translation)

