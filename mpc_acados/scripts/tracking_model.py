#!/usr/bin/env python
from casadi import *
import numpy as np
import types

def tracking_model():
    # Create a SimpleNamespace to store model information
    model = types.SimpleNamespace()
    model_name = "tracking_model"

    # ------------------------------
    # 1. Define states and controls
    # ------------------------------
    # States: 3D position and velocity, each a 3D vector
    relative_pos = MX.sym("relative_pos", 3)  # 3D position
    relative_vel = MX.sym("relative_vel", 3)  # 3D velocity
    x = vertcat(relative_pos, relative_vel)    # State vector (6 elements)

    # Control: 3D acceleration input
    u = MX.sym("u", 3)                       # Control vector

    # ------------------------------
    # 2. Define symbolic state derivatives
    # ------------------------------
    # For implicit model formulation, define state derivatives (3D each)
    relative_pos_dot = MX.sym("relative_pos_dot", 3)
    relative_vel_dot = MX.sym("relative_vel_dot", 3)
    xdot = vertcat(relative_pos_dot, relative_vel_dot)

    # ------------------------------
    # 3. No algebraic variables or model parameters
    # ------------------------------
    z = vertcat([])   # No algebraic variables
    p = vertcat([])   # No parameters

    # ------------------------------
    # 4. Define dynamics expressions
    # ------------------------------
    # Explicit dynamics:
    #   dot(pos) = vel
    #   dot(vel) = u
    f_expl = vertcat(
        relative_vel,  # Position derivative equals velocity
        u              # Velocity derivative equals control input (acceleration)
    )
    # Implicit expression: xdot - f_expl = 0
    f_impl_expr = xdot - f_expl

    # ------------------------------
    # 5. Set control input bounds
    # ------------------------------
    # For example, limit each axis acceleration to [-5, 5]
    model.u_min = np.array([-5, -5, -5])
    model.u_max = np.array([5,  5,  5])

    # ------------------------------
    # 6. Define initial state
    # ------------------------------
    # Default initial state zeros: [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z]
    model.x0 = np.zeros(6)

    # ------------------------------
    # 7. Build model structure
    # ------------------------------
    model.f_expl_expr = f_expl       # Explicit dynamics expression
    model.f_impl_expr = f_impl_expr  # Implicit dynamics expression
    model.x = x                      # State vector
    model.xdot = xdot                # State derivative symbol
    model.u = u                      # Control vector
    model.z = z                      # Algebraic variables (empty)
    model.p = p                      # Parameters (empty)
    model.name = model_name          # Model name

    return model
