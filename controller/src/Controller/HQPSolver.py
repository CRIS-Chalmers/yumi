#!/usr/bin/env python3

# Slightly modified vesion of https://github.com/ritalaezza/sot-myo/blob/akos_re/src/HQPSolver.py

import numpy as np
import quadprog

class HQPSolver(object):
    def __init__(self, stopEGM):
        #self.SoT = SoT                  # List of Task objects
        #self.slack_boundary = 1e-5      # Currently unused.
        #self.slack_ratio = 5e4         # Between qdot and w cost
        self.slack = []
        self.stopEGM = stopEGM
                
    def solve(self,  SoT=[]):
        """
        Solves the stack of tasks and returns qdot*.
        Requires the task constraints to have been updated via update_constraints() beforehand to work.
        """

        self.slack = [np.zeros((task.mdim(),)) for task in SoT]        

        # i Loop through each task in the stack, descending in the hierachy
        for i in range(0, len(SoT)):
            n_i = SoT[i].ndim()
            m_i = SoT[i].mdim()
            
            # Set up task to solve
            A, b, G, h = SoT[i].append_slack(m_i)
                
            # Set up tasks over currently solved task in stack:
            if i > 0:                
                # j Loop through all previously solved tasks:
                for j in range(0, i):
  
                    Aj, bj, Gj, hj = SoT[j].append_slack_locked(m_i, self.slack[j])
                    
                    # Add previously solved tasks with optimal slack variables, s.t. task i is solved within their null-space.
                    if Aj is not None:
                        A = np.vstack((A, Aj))
                        b = np.concatenate((b, bj), axis=0)
                    if Gj is not None:
                        G = np.vstack((G, Gj))
                        h = np.concatenate((h, hj), axis=0)

            # Set cost matrix and solve level:
            P = np.eye(n_i + m_i) * 1
            P[-m_i:, -m_i:] = SoT[i].slack_ratio() * np.eye(m_i)

            try:
                x = quadprog_solve_qp(P, np.zeros((n_i + m_i, )), G, h, A, b)
                self.slack[i] = x[n_i:]
                qd = x[:n_i]
            except:
                print('error in task number')
                print(i)
                try:
                    self.stopEGM()
                except:
                    print('failed to stop EGM, if simulation dont suport egm this is normal')
                print('Joint velocity set to 0 and egm is stoped')
                return np.zeros(n_i)

        # After solving the last task, return optimal joint velocities as control input.
        return qd


def quadprog_solve_qp(P, q=None, G=None, h=None, A=None, b=None):
    """
    Wrapper for https://pypi.org/project/quadprog/
    https://github.com/rmcgibbo/quadprog/blob/master/quadprog/quadprog.pyx
    """
    qp_G = .5 * (P + P.T)   # make sure P is symmetric
    if q is not None:
        qp_a = -q
    else:
        qp_a = np.zeros(qp_G.shape[0])
    if A is not None and G is not None:     # Mixed equality and inequality constraints
        qp_C = -np.vstack([A, G]).T
        qp_b = -np.hstack([b, h])
        meq = A.shape[0]

    elif A is not None and G is None:       # Only equality constraint (x = a reformed as -a <= x <= a) 
        qp_C = -A.T
        qp_b = -b
        meq = A.shape[0]

    else:                                   # Only ineqality constraint
        qp_C = -G.T
        qp_b = -h
        meq = 0

    return quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, meq, False)[0]
