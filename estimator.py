from __future__ import division
import numpy as np
from scipy import integrate
import rospy
from math import *
import os
import yaml

class UKF(object):
    def __init__(self):
        self.cfg=self.load_config()
        self.extract_vars()
        self.make_constants()

    def load_config(self,path=None):
        if not path:
            path=os.path.dirname(__file__) + '/config.yaml'
        try:
            with open(path, 'r') as stream:
                cfg=yaml.load(stream)
        except IOError:
            print "No config file found"
            raise
        return cfg

    def extract_vars(self):
        '''creates all variables used in the controller and estimator'''
        # center of mass
        cfg = self.cfg; 

        x_m=cfg['x_m']
        y_m=cfg['y_m']
        z_m=cfg['z_m']
        # center of buoyancy
        x_b=cfg['x_b']
        y_b=cfg['y_b']
        z_b=cfg['z_b']
        # locations of motors
        T_Lx=cfg['T_L']['x']
        T_Ly=cfg['T_L']['y']
        #  T_Lz=cfg['T_L']['z']

        T_Rx=cfg['T_R']['x']
        T_Ry=cfg['T_R']['y']
        #  T_Rz=cfg['T_R']['z']

        T_Fx=cfg['T_F']['x']
        T_Fy=cfg['T_F']['y']
        #  T_Fz=cfg['T_F']['z']
        
        T_Bx=cfg['T_B']['x']
        T_By=cfg['T_B']['y']
        #  T_Bz=cfg['T_B']['z']

        T_FRx=cfg['T_FR']['x']
        T_FRy=cfg['T_FR']['y']
        T_FRz=cfg['T_FR']['z']

        T_FLx=cfg['T_FL']['x']
        T_FLy=cfg['T_FL']['y']
        T_FLz=cfg['T_FL']['z']

        T_BRx=cfg['T_BR']['x']
        T_BRy=cfg['T_BR']['y']
        T_BRz=cfg['T_BR']['z']

        T_BLx=cfg['T_BL']['x']
        T_BLy=cfg['T_BL']['y']
        T_BLz=cfg['T_BL']['z']

        self.dx_b=x_b-x_m
        self.dy_b=y_b-y_m
        self.dz_b=z_b-z_m

        self.dx_F=T_Fx-x_m
        self.dy_F=T_Fy-y_m

        self.dx_R=T_Rx-x_m
        self.dy_R=T_Ry-y_m

        self.dx_B=T_Bx-x_m
        self.dy_B=T_By-y_m

        self.dx_L=T_Lx-x_m
        self.dy_L=T_Ly-y_m

        self.dx_FR=T_FRx-x_m
        self.dy_FR=T_FRy-y_m
        self.dz_FR=T_FRz-z_m

        self.dx_FL=T_FLx-x_m
        self.dy_FL=T_FLy-y_m
        self.dz_FL=T_FLz-z_m

        self.dx_BR=T_BRx-x_m
        self.dy_BR=T_BRy-y_m
        self.dz_BR=T_BRz-z_m

        self.dx_BL=T_BLx-x_m
        self.dy_BL=T_BLy-y_m
        self.dz_BL=T_BLz-z_m

        # mass of sub (kg)
        self.m=cfg['m']
        # gravity (m/s**2)
        self.g=cfg['g']
        # density of water (kg/m**3)
        self.rho=cfg['rho']
        # buoyancy force
        self.Fb=cfg['Fb']
        # moments of inertia
        self.Ixx=cfg['Ixx']
        self.Iyy=cfg['Iyy']
        self.Izz=cfg['Izz']
        # Drag coefficients
        self.Cd_x=cfg['Cd_x']
        self.Cd_y=cfg['Cd_y']
        self.Cd_z=cfg['Cd_z']

        # top, bottom, left, right
        # intersection of y along x axis
        self.Cd_xT=cfg['Cd_xT']
        self.Cd_xB=cfg['Cd_xB']
        # intersection of z along x axis
        self.Cd_xR=cfg['Cd_xR']
        self.Cd_xL=cfg['Cd_xL']

        # top, bottom, front, back
        # intersectoin of x along y axis
        self.Cd_yT=cfg['Cd_yT']
        self.Cd_yB=cfg['Cd_yB']
        # intersectoin of z along y axis
        self.Cd_yF=cfg['Cd_yF']
        self.Cd_yBa=cfg['Cd_yBa']

        # left, righ, front, back
        # intersectoin of x along z axis
        self.Cd_zL=cfg['Cd_zL']
        self.Cd_zR=cfg['Cd_zR']
        # intersectoin of y along z axis
        self.Cd_zF=cfg['Cd_zF']
        self.Cd_zB=cfg['Cd_zB']

        # Surface areas
        self.A_x=cfg['A_x']
        self.A_y=cfg['A_y']
        self.A_z=cfg['A_z']

        # top, bottom, left, right
        self.A_xT=cfg['A_xT']
        self.A_xB=cfg['A_xB']
        self.A_xR=cfg['A_xR']
        self.A_xL=cfg['A_xL']

        # top, bottom, front, back
        self.A_yT=cfg['A_yT']
        self.A_yB=cfg['A_yB']
        self.A_yF=cfg['A_yF']
        self.A_yBa=cfg['A_yBa']

        # left, righ, front, back
        self.A_zL=cfg['A_zL']
        self.A_zR=cfg['A_zR']
        self.A_zF=cfg['A_zF']
        self.A_zB=cfg['A_zB']

        # Moment arm lengths
        # how to read:
        # dz_xT = distance along z axis to center of top slice viewed in the x direction
        self.dz_xT=cfg['dz_xT']
        self.dz_xB=cfg['dz_xB']
        self.dy_xR=cfg['dy_xR']
        self.dy_xL=cfg['dy_xL']

        self.dz_yT=cfg['dz_yT']
        self.dz_yB=cfg['dz_yB']
        self.dx_yF=cfg['dx_yF']
        self.dx_yBa=cfg['dx_yBa']

        self.dy_zL=cfg['dy_zL']
        self.dy_zR=cfg['dy_zR']
        self.dx_zF=cfg['dx_zF']
        self.dx_zB=cfg['dx_zB']

    def make_constants(self):
        '''function designed to compute common strings
        of constants used in EOM to speed up
        computation
        Input: constants-constants from modeling/config
        file'''

        self.phi_R=cos(atan(self.dz_FR/self.dy_FR))*sqrt(self.dz_FR**2+self.dy_FR**2)
        self.phi_L=cos(atan(self.dz_FL/self.dy_FL))*sqrt(self.dz_FL**2+self.dy_FL**2)
        self.phi_bz=cos(atan(self.dy_b/self.dz_b))*sqrt(self.dz_b**2+self.dy_b**2)
        self.phi_by=cos(atan(self.dz_b/self.dy_b))*sqrt(self.dz_b**2+self.dy_b**2)

        self.theta_F=cos(atan(self.dz_FR/self.dx_FR))*sqrt(self.dz_FR**2+self.dx_FR**2)
        self.theta_B=cos(atan(self.dz_BL/self.dx_BL))*sqrt(self.dz_BL**2+self.dx_BL**2)
        self.theta_bx=cos(atan(self.dz_b/self.dx_b))*sqrt(self.dz_b**2+self.dx_b**2)
        self.theta_bz=cos(atan(self.dx_b/self.dz_b))*sqrt(self.dz_b**2+self.dx_b**2)

        self.psi_L=cos(atan(self.dx_L/self.dy_L))*sqrt(self.dy_R**2+self.dx_R**2)
        self.psi_R=cos(atan(self.dx_R/self.dy_R))*sqrt(self.dy_L**2+self.dx_L**2)
        self.psi_F=cos(atan(self.dy_F/self.dx_F))*sqrt(self.dy_F**2+self.dx_F**2)
        self.psi_B=cos(atan(self.dy_B/self.dx_B))*sqrt(self.dy_B**2+self.dx_B**2)
        self.psi_bx=cos(atan(self.dy_b/self.dx_b))*sqrt(self.dy_b**2+self.dx_b**2)
        self.psi_by=cos(atan(self.dx_b/self.dy_b))*sqrt(self.dy_b**2+self.dx_b**2)

    def make_nonLinearFunctions(self):
        '''create lambda functions for non-linear state equations'''
        #Drag Forces
        self.F_D_x = lambda x : (self.rho/2)*(self.Cd_x*abs(cos(x[10])*cos(x[8]))*self.A_x + self.Cd_y*abs(sin(x[10]))*self.A_y + self.Cd_z*abs(sin(x[10]))*self.A_z)*(x[1]**2); 
        self.F_D_y = lambda x : (self.rho/2)*(self.Cd_x*abs(sin(x[10]))*self.A_x + self.Cd_y*abs(cos(x[10])*cos(x[6]))*self.A_y + self.Cd_z*abs(sin(x[6]))*self.A_z)*(x[3]**2); 
        self.F_D_z = lambda x : (self.rho/2)*(self.Cd_x*abs(sin(x[8]))*self.A_x + self.Cd_y*abs(sin(x[6]))*A_y + self.Cd_z*abs(cos(x[6])*cos(x[8]))*self.A_z)*(x[5]**2); 

        self.F_D_phi = lambda x : (self.rho/(2*self.Ixx))*(self.Cd_z*self.A_zL*self.dy_zL(x[7]*self.dy_zL + x[1]*abs(sin(x[8])) + x[3]*abs(sin(x[6])) + x[5]*abs(cos(x[6])*cos(x[8])) + \


        #State Derivatives
        self.xdotdot = lambda x,u : ((u[0]+u[1])*(cos(x[10]))*(cos(x[8])) + (u[2]+u[3])*(-sin(x[10])) + (u[4]+u[5]+u[6]+u[7])*(sin(x[8])) - self.F_D_x(x))/self.m; 
        self.ydotdot = lambda x,u : ((u[0]+u[1])*(sin(x[10])) + (u[2]+u[3])*(cos(x[10])*cos(x[6])) - (u[4]+u[5]+u[6]+u[7])*sin(x[6]) - self.F_D_y(x))/self.m; 
        self.zdotdot = lambda x,u : ((u[0]+u[1])*(-sin(x[8])) + (u[2]+u[3])*sin(x[6]) + (u[4]+u[5]+u[6]+u[7])*(cos(x[6])*cos(x[8])) - F_D_z - self.Fb)/self.m; 

        self.phidotdot = lambda x,u : (1/self.Ixx)*((u[5]+u[7])*self.phi_R - (u[4]+u[6])*self.phi_L) - self.F_D_phi(x) + (self.Fb/self.Ixx)*(self.phi_bz*cos(x[8])*sin(x[6]) - self.phi_bz*(cos(x[8])*cos(x[6]))); 
        self.thetadotdot = lambda x,u : (1/self.Iyy)*(-(u[7]+u[6])*self.theta_B + (u[4]+u[5])*self.theta_F) - self.F_D_theta(x) + (self.Fb/self.Iyy)*(self.theta_bx*cos(x[8])*cos(x[6]) + self.theta_bz*sin(x[8])); 
        self.psidotdot = lambda x,u : (1/self.Izz)*(u[0]*self.psi_L - u[1]*self.psi_R + u[2]*self.psi_F + u[3](self.psi_B)) - self.F_D_psi(x) + (self.Fb/self.Izz)*(self.psi_bx*cos(x[8])*cos(x[6]) - self.psi_by*sin(x[8])); 

        self.accels = lambda x,u : [self.xdotdot(x,u),self.ydotdot(x,u),self.zdotdot(x,u),self.phidotdot(x,u),self.thetadotdot(x,u),self.psidotdot(x,u)]; 


    def ROS_sensors(self):
        '''get sensor measurements from ROS'''
        pass
    
    def propagate_pred(self,point):
        '''propagates a single sigma point according to
        the non-linear dynamics of the system for the 
        prediction step'''
        pass

    def prediction(self,points):
        '''given a set of sigma points, combine to produce
        a predicted mean and covariance'''
        pass

    def propagate_meas(self,point):
        '''propagates a single sigma point according to
        the non-linear dynamics of the system for the 
        measurement step'''
        pass

    def state_update(self):
        '''calculate kalman gain and perform final
        state estimate update for the non-linear dynamics'''
        pass


def testNonLinear():
    x = [0,0,0,0,0,0,0,0,0,0,0,0]; 
    u = [.2,1,0,0,0,0,0,0]; 

    fil = UKF(); 
    fil.make_nonLinearFunctions(); 

    a = fil.xdotdot(x,u); 
    print(a); 


if __name__ == '__main__':
    testNonLinear(); 
