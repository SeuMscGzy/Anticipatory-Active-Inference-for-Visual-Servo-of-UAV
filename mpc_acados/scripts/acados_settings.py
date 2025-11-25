from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from tracking_model import tracking_model
import scipy.linalg
import numpy as np

def acados_settings(Tf, N):

    # create OCP object to formulate the optimization
    ocp = AcadosOcp()

    # export model
    model = tracking_model()

    # define acados ODE 
    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x = model.x
    model_ac.xdot = model.xdot
    model_ac.u = model.u
    model_ac.z = model.z
    model_ac.p = model.p
    model_ac.name = model.name
    ocp.model = model_ac

    # dimensions 
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx 

    # discretization 
    ocp.dims.N = N

    # set cost 
    Q = np.diag([3.1, 3.1, 3.1, 1, 1, 1])
    R = 0.1 * np.eye(nu)

    Qe = Q

    ocp.cost.cost_type   = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    # unscale = N / Tf
    # ocp.cost.W   = unscale * scipy.linalg.block_diag(Q,R)
    # ocp.cost.W_e = Qe / unscale
    
    ocp.cost.W   = scipy.linalg.block_diag(Q,R)
    ocp.cost.W_e = Qe

    Vx = np.zeros((ny, nx))
    Vx[:nx, :] = np.eye(nx)
    ocp.cost.Vx = Vx

    Vu = np.zeros((ny, nu))

    Vu[nx:nx+nu, :] = np.eye(nu)
    ocp.cost.Vu = Vu

    Vx_e = np.zeros((ny_e, nx))
    Vx_e[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = Vx_e
    
    # Initial reference trajectory (can be overwritten during the simulation if required)
    x_ref = np.array([0, 0, 0, 0, 0, 0])
    ocp.cost.yref   = np.concatenate((x_ref, np.array([0, 0, 0])))
    ocp.cost.yref_e = x_ref

    # set constraints
    ocp.constraints.lbu   = model.u_min
    ocp.constraints.ubu   = model.u_max
    ocp.constraints.idxbu = np.array([0, 1, 2])

    # set initial condition
    ocp.constraints.x0 = model.x0

    # set QP solver and integration
    ocp.solver_options.tf = Tf
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.nlp_solver_max_iter = 300
    ocp.solver_options.tol = 1e-3

    # create ocp solver 
    acados_solver = AcadosOcpSolver(ocp, json_file=(model_ac.name + "_" + "acados_ocp.json"))

    return model, acados_solver