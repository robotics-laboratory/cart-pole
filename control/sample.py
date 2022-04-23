def cartpole_generate_data():
    fig, ax = plt.subplots(2, 2)
    ax[0,0].set_xlabel('t')
    ax[0,0].set_ylabel('x')
    ax[1,0].set_xlabel('t')
    ax[1,0].set_ylabel('theta')
    ax[0,1].set_xlabel('t')
    ax[0,1].set_ylabel('xdot')
    ax[1,1].set_xlabel('t')
    ax[1,1].set_ylabel('thetadot')

    data = []

    for i in range(1):
        ts = np.arange(0.02, 20, 0.02)
        #utraj = PiecewisePolynomial.CubicShapePreserving(ts, ts*10*np.cos((2*ts)**1.5).reshape((1,-1)))
        #utraj = PiecewisePolynomial.CubicShapePreserving(ts, 2*(np.cos(ts**2) - 2*(ts**2)*np.sin(ts**2)).reshape((1,-1)))
        utraj = PiecewisePolynomial.CubicShapePreserving(ts, 3 * (
            np.cos(ts ** 1.5)/(ts ** 0.5) - 2.73 * (ts ** 1) * np.sin(ts ** 1.5)
        ).reshape((1,-1)))
        builder = DiagramBuilder()

        plant = builder.AddSystem(MultibodyPlant(0.0))
        file_name = FindResource("models/cartpole.urdf")
        Parser(plant).AddModelFromFile(file_name)
        plant.Finalize()

        usys = builder.AddSystem(TrajectorySource(utraj))
        builder.Connect(usys.get_output_port(), plant.get_actuation_input_port())

        logger = LogVectorOutput(plant.get_state_output_port(), builder)
        diagram = builder.Build()

        simulator = Simulator(diagram)
        simulator.AdvanceTo(ts[-1])

        log = logger.FindLog(simulator.get_context())
        data.append(DataFrame({
            't': log.sample_times(),
            'u': utraj.vector_values(log.sample_times())[0,:],
            'x': log.data()[0,:],
            'theta': log.data()[1,:],
            'xdot': log.data()[2,:],
            'thetadot': log.data()[3,:],
            }))

        ax[0,0].plot(data[i].t, data[i].x.T)
        ax[1,0].plot(data[i].t, data[i].theta.T)
        ax[0,1].plot(data[i].t, data[i].xdot.T)
        ax[1,1].plot(data[i].t, data[i].thetadot.T)

    return data

data = cartpole_generate_data()





def cartpole_estimate_lumped_parameters(data):
    plant = MultibodyPlant(0.0)
    file_name = FindResource("models/cartpole.urdf")
    Parser(plant).AddModelFromFile(file_name)
    plant.Finalize()

    sym_plant = plant.ToSymbolic()
    sym_context = sym_plant.CreateDefaultContext()

    # State variables
    q = MakeVectorVariable(2, "q")
    v = MakeVectorVariable(2, "v")
    vd = MakeVectorVariable(2, "\dot{v}")
    tau = MakeVectorVariable(1, "u")

    # Parameters
    mc = Variable("mc")
    mp = Variable("mp")
    l = Variable("l")

    sym_plant.get_actuation_input_port().FixValue(sym_context, tau)
    sym_plant.SetPositions(sym_context, q)
    sym_plant.SetVelocities(sym_context, v)

    cart = sym_plant.GetBodyByName("cart")
    cart.SetMass(sym_context, mc)
    pole = sym_plant.GetBodyByName("pole")
    inertia = SpatialInertia_[Expression](mp, [0, 0, -l], UnitInertia_[Expression](l*l,l*l,0))
    pole.SetSpatialInertiaInBodyFrame(sym_context, inertia)

    # TODO: Set the joint damping pending resolution of drake #14405
    # x = sym_plant.GetJointByName("x")
    # theta = sym_plant.GetJointByName("theta")

    derivatives = sym_context.Clone().get_mutable_continuous_state()
    derivatives.SetFromVector(np.hstack((0*v, vd)))
    residual = sym_plant.CalcImplicitTimeDerivativesResidual(
        sym_context, derivatives)

    print('Symbolic acceleration residuals: ')
    display(Math(ToLatex(residual[2:], 2)))

    W, alpha, w0 = DecomposeLumpedParameters(residual[2:], [mc, mp, l])

    M = data[0].t.shape[0] - 1
    MM = 2*M*len(data)
    N = alpha.size
    Wdata = np.zeros((MM, N))
    w0data = np.zeros((MM, 1))
    offset = 0
    for d in data:
        for i in range(M):
            h = d.t[i+1]-d.t[i]
            env = {
                q[0]: d.x[i],
                q[1]: d.theta[i],
                v[0]: d.xdot[i],
                v[1]: d.thetadot[i],
                vd[0]: (d.xdot[i+1]-d.xdot[i])/h,
                vd[1]: (d.thetadot[i+1]-d.thetadot[i])/h,
                tau[0]: d.u[i],
            }

            Wdata[offset:offset+2,:] = Evaluate(W, env)
            w0data[offset:offset+2] = Evaluate(w0, env)
            offset += 2

    alpha_fit = np.linalg.lstsq(Wdata, -w0data, rcond=None)[0]
    alpha_true = Evaluate(alpha, {mc: 10, mp: 1, l: .5})

    print('Estimated Lumped Parameters:')
    for i in range(len(alpha)):
        print(f"{alpha[i]}.  URDF: {alpha_true[i,0]},  Estimated: {alpha_fit[i,0]}")

cartpole_estimate_lumped_parameters(data)







from common import Config, State, generate_pyplot_animation
from control import BalanceLQRControl, Trajectory, TrajectoryLQRControl
from simulator import CartPoleSimulator

import matplotlib.pyplot as plt
import numpy as np

from IPython.display import HTML

"""
Calibration steps!

1. TRACK'S LENGTH *update*
    1. Move max to the right
    2. Move max to the left
    3. *update* hard_max_position = covered distance
    4. *update* max_position = 0.9 * hard_max_position
    5. *update* zero_point (in State?)

2. HOMING

3. POLE'S MASS AND LENGTH *update*
    1. Some sin-looking accelerated trajectory 
    2. *update* pole_mass
    3. *update* pole_length
    
4. HOMING
"""


class Calibration:
    
    max_position_calibr: float = 0  # m
    hard_max_position_calibr: float = 0  # m
        
    pole_length_calibr: float = 0  # m
    pole_mass_calibr: float = 0  # kg

    
    def __init__(self, config):
        
        
        
    def __call__(self, state):
        
        return 
    