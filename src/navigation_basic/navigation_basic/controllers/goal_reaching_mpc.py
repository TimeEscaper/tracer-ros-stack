import do_mpc
import casadi
import numpy as np

from typing import Tuple


class DoMPCControllerGoalReaching:

    def __init__(self, 
                 dt: float,
                 u_linear_bound: Tuple[float, float],
                 u_angular_bound: Tuple[float, float],
                 horizon: int = 20,
                 R_diag: Tuple[float, float] = (0.1, 0.1),
                 Q_diag: Tuple[float, float] = (1., 1.)):
        self._dt = dt
        self._horizon = horizon

        self._model = do_mpc.model.Model("discrete")

        # State variables: x, y and theta (orientation)
        pose_x = self._model.set_variable(var_type='_x', var_name='pose_x')
        pose_y = self._model.set_variable(var_type='_x', var_name='pose_y')
        pose_theta = self._model.set_variable(var_type='_x', var_name='pose_theta')

        # Control variables: linear velocity and angular velocity
        u_v = self._model.set_variable(var_type='_u', var_name='u_linear')
        u_omega = self._model.set_variable(var_type='_u', var_name='u_angular')

        # Template variables: position of the goal
        goal_x = self._model.set_variable('_p', 'goal_x')
        goal_y = self._model.set_variable('_p', 'goal_y')

        # Transition equations
        self._model.set_rhs('pose_x', pose_x + u_v * casadi.cos(pose_theta) * self._dt)
        self._model.set_rhs('pose_y', pose_y + u_v * casadi.sin(pose_theta) * self._dt)
        self._model.set_rhs('pose_theta', pose_theta + u_omega * self._dt)

        # Do the final model setup
        self._model.setup()

        # # Cost functions
        # R = np.diag(R_diag)
        # Q = np.diag(Q_diag)
        # x = self._model._x.cat[:2]
        # u = self._model._u.cat
        # goal = self._model._p.cat
        # self._model.set_expression(expr_name='stage_cost',
        #                            expr=u.T @ R @ u)
        # self._model.set_expression(expr_name='final_cost',
        #                            expr=(x - goal).T @ Q @ (x - goal))
        
        # General settings for the MPC solver
        self._mpc = do_mpc.controller.MPC(self._model)
        setup_mpc = {
            'n_robust': 0,
            'n_horizon': horizon,
            't_step': self._dt,
            'state_discretization': 'discrete',
            'store_full_solution': True,
            'nlpsol_opts': {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}
        }
        self._mpc.set_param(**setup_mpc)
        
        # Cost functions
        R = np.diag(R_diag)
        print(R)
        Q = np.diag(Q_diag)
        x = self._model._x.cat[:2]
        u = self._model._u.cat
        goal = self._model._p.cat
        self._mpc.set_objective(lterm=u.T @ R @ u,
                                mterm=(x - goal).T @ Q @ (x - goal))
        
        # Set bounds on variables
        self._mpc.bounds['lower', '_x', 'pose_theta'] = -np.pi
        self._mpc.bounds['upper', '_x', 'pose_theta'] = np.pi
        self._mpc.bounds['lower', '_u', 'u_linear'] = u_linear_bound[0]
        self._mpc.bounds['upper', '_u', 'u_linear'] = u_linear_bound[1]
        self._mpc.bounds['lower', '_u', 'u_angular'] = u_angular_bound[0]
        self._mpc.bounds['upper', '_u', 'u_angular'] = u_angular_bound[1]

        # Rterms for more smooth movement
        self._mpc.set_rterm(u_linear=1e-2, u_angular=1e-2)

        # self._mpc.set_uncertainty_values(goal_x=np.array([goal[0]]),
        #                                  goal_y=np.array([goal[1]]),
        #                                  goal_theta=np.array([goal[2]]))
        self._mpc.set_uncertainty_values(goal_x=np.array([0.]),
                                         goal_y=np.array([0.]))
        
        # Final MPC solver setup
        self._mpc.setup()

    def predict(self, x_current: Tuple[float, float, float], goal: Tuple[float, float]) -> np.ndarray:
        x_current = np.array(x_current)
        self._mpc.set_uncertainty_values(goal_x=np.array([goal[0]]),
                                         goal_y=np.array([goal[1]]))
        self._mpc.x0 = x_current
        self._mpc.set_initial_guess()

        u0 = self._mpc.make_step(x_current)
        u0 = u0.flatten()

        return u0
