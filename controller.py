from __future__ import division
import numpy as np
import control
import rospy
from estimator import UKF
from math import *
import scipy.linalg


class Controller(UKF):
    def ROS_estimate(self):
        '''retrieves state estimate from UKF through
        ROS'''
        pass

    def linearize_matricies(self,x,u,dt):
        '''creates discritized linearized F,G,H&M matricies
        Input: x-linearization point
        u-linearization inputs
        dt-discrete interval'''
        def discritize(A,B,dt):
            n=A.shape[0]
            r=B.shape[1]
            Ahat=np.zeros([n+r,n+r])
            Ahat[0:n,0:n]=A
            Ahat[0:n,n:]=B
            FG0I=scipy.linalg.expm(Ahat*dt)
            F=FG0I[0:n,0:n]
            G=FG0I[0:n,n:]
            return A,B

        dxdotdot_dxdot=-(self.A_x*self.Cd_x*self.rho*x[1])/self.m
        dxdotdot_dtheta=-self.g*cos(x[10])*cos(x[8])+((self.Fb*cos(x[10])*cos(x[8]))/self.m)
        dxdotdot_dpsi=self.g*sin(x[10])*sin(x[8])-((self.Fb*sin(x[10])*sin(x[8]))/self.m)
        dxdotdot_dTL=1/self.m
        dxdotdot_dTR=1/self.m

        dydotdot_dydot=-(self.A_y*self.Cd_y*self.rho*x[3])/self.m
        dydotdot_dphi=self.g*cos(x[6])*cos(x[10])-((self.Fb*cos(x[10])*cos(x[6]))/self.m)
        dydotdot_dpsi=-self.g*sin(x[6])*sin(x[10])+((self.Fb*sin(x[10])*sin(x[6]))/self.m)
        dydotdot_dTF=1/self.m
        dydotdot_dTB=1/self.m

        dzdotdot_dzdot=-(self.A_z*self.Cd_z*self.rho*x[5])/self.m
        dzdotdot_dphi=-self.g*cos(x[8])*sin(x[6])+((self.Fb*cos(x[8])*sin(x[6]))/self.m)
        dzdotdot_dtheta=-self.g*cos(x[6])*sin(x[8])+((self.Fb*cos(x[6])*sin(x[8]))/self.m)
        dzdotdot_dTFL=1/self.m
        dzdotdot_dTFR=1/self.m
        dzdotdot_dTBL=1/self.m
        dzdotdot_dTBR=1/self.m

        dphidotdot_dydot=-.5*self.Ixx*self.rho*(2*self.A_yB*self.Cd_yB*self.dz_yB* \
                (self.dz_yB*x[7]+x[3])+2*self.A_yT*self.Cd_yT*self.dz_yT*(self.dz_yT*x[7]+x[3]))
        dphidotdot_dzdot=-.5*self.Ixx*self.rho*(2*self.A_zL*self.Cd_zL*self.dy_zL* \
                (self.dy_zL*x[7]+x[5])+2*self.A_zR*self.Cd_zR*self.dy_zR*(self.dy_zR*x[7]+x[5]))
        dphidotdot_dphi=(self.Fb/self.Ixx)*(self.phi_bz*cos(x[6])*cos(x[8])+self.phi_by*cos(x[8])*sin(x[6]))
        dphidotdot_dphidot=-.5*self.Ixx*self.rho*(2*self.A_yB*self.Cd_yB*self.dz_yB**2* \
                (self.dz_yB*x[7]+x[3])+2*self.A_yT*self.Cd_yT*self.dz_yT**2*(self.dz_yT*x[7]+x[3])+ \
                2*self.A_zL*self.Cd_zL*self.dy_zL**2*(self.dy_zL*x[7]+x[5])+2*self.A_zR*self.Cd_zR* \
                self.dy_zR**2*(self.dy_zR*x[7]+x[5]))
        dphidotdot_dtheta=(self.Fb/self.Ixx)*(self.phi_by*cos(x[6])*sin(x[8])-self.phi_bz*sin(x[6])*sin(x[8]))
        dphidotdot_dTFL=-self.phi_L/self.Ixx
        dphidotdot_dTFR=self.phi_R/self.Ixx
        dphidotdot_dTBL=-self.phi_L/self.Ixx
        dphidotdot_dTBR=self.phi_R/self.Ixx

        dthetadotdot_dxdot=-.5*self.Iyy*self.rho*(2*self.A_xB*self.Cd_xB*self.dz_xB* \
                (self.dz_xB*x[9]+x[1])+2*self.A_xT*self.Cd_xT*self.dz_xT*(self.dz_xT*x[9]+x[1]))
        dthetadotdot_dzdot=-.5*self.Iyy*self.rho*(2*self.A_zB*self.Cd_zB*self.dx_zB* \
                (self.dx_zB*x[9]+x[5])+2*self.A_zF*self.Cd_zF*self.dx_zF*(self.dx_zF*x[9]+x[5]))
        dthetadotdot_dphi=(-self.Fb/self.Iyy)*(self.theta_bx*cos(x[8])*sin(x[6]))
        dthetadotdot_dtheta=(self.Fb/self.Iyy)*(self.theta_bz*cos(x[8])-self.theta_bx*cos(x[6])*sin(x[8]))
        dthetadotdot_dthetadot=-.5*self.Iyy*self.rho*(2*self.A_xB*self.Cd_xB*self.dz_xB**2* \
                (self.dz_xB*x[9]+x[1])+2*self.A_xT*self.Cd_xT*self.dz_xT**2*(self.dz_xT*x[9]+x[1])+ \
                2*self.A_zB*self.Cd_zB*self.dx_zB**2*(self.dx_zB*x[9]+x[5])+2*self.A_zF*self.Cd_zF* \
                self.dx_zF**2*(self.dx_zF*x[9]+x[5]))
        dthetadotdot_dTFL=self.theta_F/self.Iyy
        dthetadotdot_dTFR=self.theta_F/self.Iyy
        dthetadotdot_dTBL=-self.theta_B/self.Iyy
        dthetadotdot_dTBR=-self.theta_B/self.Iyy
        
        dpsidotdot_dxdot=-.5*self.Izz*self.rho*(2*self.A_xL*self.Cd_xL*self.dy_xL* \
                (self.dy_xL*x[11]+x[1])+2*self.A_xR*self.Cd_xR*self.dy_xR*(self.dy_xR*x[11]+x[1]))
        dpsidotdot_dydot=-.5*self.Izz*self.rho*(2*self.A_yBa*self.Cd_yBa*self.dx_yBa* \
                (self.dx_yBa*x[11]+x[3])+2*self.A_yF*self.Cd_yF*self.dx_yF*(self.dx_yF*x[11]+x[3]))
        dpsidotdot_dphi=(self.Fb/self.Izz)*(self.psi_bx*cos(x[8])*sin(x[6]))
        dpsidotdot_dtheta=(self.Fb/self.Izz)*(-self.psi_by*cos(x[8])-self.psi_bx*sin(x[6])*sin(x[8]))
        dpsidotdot_dpsidot=-.5*self.Izz*self.rho*(2*self.A_xL*self.Cd_xL*self.dy_xL**2* \
                (self.dy_xL*x[11]+x[1])+2*self.A_xR*self.Cd_xR*self.dy_xR**2*(self.dy_xR*x[11]+x[1])+ \
                2*self.A_yBa*self.Cd_yBa*self.dx_yBa**2*(self.dx_yBa*x[11]+x[3])+2*self.A_yF*self.Cd_yF* \
                self.dx_yF**2*(self.dx_yF*x[11]+x[3]))
        dpsidotdot_dTL=self.psi_L/self.Izz
        dpsidotdot_dTR=-self.psi_R/self.Izz
        dpsidotdot_dTF=self.psi_F/self.Izz
        dpsidotdot_dTB=-self.psi_B/self.Izz

        # A should be a 12x12
        # B should be a 12x8
        # C should be a 10x12 (12x12 with DVL)
        # D should be a 10x8 (12x8 with DVL)
        A=np.array([[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, dxdotdot_dxdot, 0, 0, 0, 0, 0, 0, dxdotdot_dtheta, 0, dxdotdot_dpsi, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, dydotdot_dydot, 0, 0, dydotdot_dphi, 0, 0, 0, dydotdot_dpsi, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, dzdotdot_dzdot, dzdotdot_dphi, 0, dzdotdot_dtheta, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, dphidotdot_dydot, 0, dphidotdot_dzdot, dphidotdot_dphi, dphidotdot_dphidot, dphidotdot_dtheta, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, dthetadotdot_dxdot, 0, 0, 0, dthetadotdot_dzdot, dthetadotdot_dphi, 0, dthetadotdot_dtheta, dthetadotdot_dthetadot, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            [0, dpsidotdot_dxdot, 0, dpsidotdot_dydot, 0, 0, dpsidotdot_dphi, 0, dpsidotdot_dtheta, 0, dxdotdot_dpsi, dpsidotdot_dpsidot]])

        B=np.array([[0, 0, 0, 0, 0, 0, 0, 0],
            [dxdotdot_dTL, dxdotdot_dTR, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, dydotdot_dTF, dydotdot_dTB, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, dzdotdot_dTFL, dzdotdot_dTFR, dzdotdot_dTBL, dzdotdot_dTBR],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, dphidotdot_dTFL, dphidotdot_dTFR, dphidotdot_dTBL, dphidotdot_dTBR],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, dthetadotdot_dTFL, dthetadotdot_dTFR, dthetadotdot_dTBL, dthetadotdot_dTBR],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [dpsidotdot_dTL, dpsidotdot_dTR, dpsidotdot_dTF, dpsidotdot_dTB, 0, 0, 0, 0]])

        C=np.array([[0, dxdotdot_dxdot, 0, 0, 0, 0, 0, 0, dxdotdot_dtheta, 0, dxdotdot_dpsi, 0],
            [0, 0, 0, dydotdot_dydot, 0, 0, dydotdot_dphi, 0, 0, 0, dydotdot_dpsi, 0],
            [0, 0, 0, 0, 0, dzdotdot_dzdot, dzdotdot_dphi, 0, dzdotdot_dtheta, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]])

        D=np.array([[dxdotdot_dTL, dxdotdot_dTR, 0, 0, 0, 0, 0, 0],
            [0, 0, dydotdot_dTF, dydotdot_dTB, 0, 0, 0, 0],
            [0, 0, 0, 0, dzdotdot_dTFL, dzdotdot_dTFR, dzdotdot_dTBL, dzdotdot_dTBR],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0]])

        if np.linalg.matrix_rank(control.ctrb(A,B))<12:
            print "Sytem is not fully controllable"
        else:
            print "Sytem is fully controllable"
        if np.linalg.matrix_rank(control.obsv(A,C))<12:
            print "System is not fully observable"
        else:
            print "Sytem is fully observable"
        A,B=discritize(A,B,dt)
        # TODO: for some reason this reduces A from 12x12 to 10x10
        return control.ss(A,B,C,D)

    def LQR(self,state_space):
        def dlqr(A,B,Q,R):
            """Solve the discrete time lqr controller.
            x[k+1] = A x[k] + B u[k]
            cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
            """
            #  http://www.mwm.im/lqr-controllers-with-python/
            #ref Bertsekas, p.151

            #first, try to solve the ricatti equation
            S = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
            #compute the LQR gain
            K = np.matrix(scipy.linalg.inv(B.T*S*B+R)*(B.T*S*A))
            eigVals, eigVecs = scipy.linalg.eig(A-B*K)
            return K, S, eigVals
        #TODO: this can be cleaned up. Easy way to specify different controllers
        rho=0.05
        x_max=5
        x_dot_max=0.1
        y_max=5
        y_dot_max=0.1
        z_max=5
        z_dot_max=0.1
        phi_max=5
        phi_dot_max=0.1
        theta_max=5
        theta_dot_max=0.1
        psi_max=5
        psi_dot_max=0.1
        Q=np.array([1/x_max**2,1/x_dot_max**2,1/y_max**2,1/y_dot_max**2,
            1/z_max**2,1/z_dot_max**2,1/phi_max**2,1/phi_dot_max**2,
            1/theta_max**2,1/theta_dot_max**2,1/psi_max**2,1/psi_dot_max**2,])*np.eye(12)
        R=rho*np.eye(8)
        #TODO: This needs to be removed, A matrix should be 12x12
        Q=Q[0:10,0:10]
        #  print state_space.A.shape
        #  print Q.shape
        K,S,E=dlqr(state_space.A,state_space.B,Q,R)
        print K
        return K

    def feed_forward(self,state_space,K):
        '''creates the feed forward matrix F to compute
        control inputs using a desired state'''
        Gcl=-state_space.C*np.linalg.inv(state_space.A-state_space.B*K)*state_space.B
        # this is done because the size of the matrix must map our measurements to our inputs 
        F=np.linalg.lstsq(Gcl,np.eye(9))
        if np.linalg.matrix_rank(control.ctrb(state_space.A-state_space.B*K,state_space.B*F))<12:
            print "Sytem is not fully controllable"
        else:
            print "Sytem is fully controllable"
        if np.linalg.matrix_rank(control.obsv(state_space.A-state_space.B*K,state_space.C-state_space.D*K))<12:
            print "System is not fully observable"
        else:
            print "Sytem is fully observable"

        return F

    def sub_planner(self, points):
        '''given a set of points to hit on a trajectory,
        determine the desired state w/velocity to send
        to the controller'''
        pass

    def control_output(self,x,x_desired,type='moving'):
        '''uses a previously computed gain K to 
        create a control input for the motors
        x-current state
        x_desired-desired state
        type-which controller to use'''
        #TODO: use stored F and K matricies for a specific controller
        u=-K*x+F*x_desired
    
    def test_controller(self):
        '''tests such as saturation and simulated responses'''
        pass

if __name__ == '__main__':
    a=Controller()
    x=[0,0,0,0,0,0,0,0,0,0,0,0]
    u=[0,0,0,0,0,0,0,0]
    dt=1/25
    state_space=a.linearize_matricies(x,u,dt)
    K=a.LQR(state_space)
    F=a.feed_forward(state_space,K)
