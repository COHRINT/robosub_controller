from __future__ import division
import numpy as np
from scipy import integrate
from scipy.linalg import sqrtm
from scipy.linalg import det
#import rospy
from math import *
import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy
import time;


class UKF(object):
    def __init__(self):
        self.cfg=self.load_config()
        self.extract_vars()
        self.make_constants()
        self.make_nonLinearFunctions(); 
        self.makeNoise(); 

        self.dt = 1/25

    def load_config(self,path=None):
        if not path:
            path=os.path.dirname(__file__) + '/../config.yaml'
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
        T_Lf=cfg['T_L']['f']

        T_Rx=cfg['T_R']['x']
        T_Ry=cfg['T_R']['y']
        #  T_Rz=cfg['T_R']['z']
        T_Rf=cfg['T_R']['f']

        T_Fx=cfg['T_F']['x']
        T_Fy=cfg['T_F']['y']
        #  T_Fz=cfg['T_F']['z']
        T_Ff=cfg['T_F']['f']
        
        T_Bx=cfg['T_B']['x']
        T_By=cfg['T_B']['y']
        #  T_Bz=cfg['T_B']['z']
        T_Bf=cfg['T_B']['f']

        T_FRx=cfg['T_FR']['x']
        T_FRy=cfg['T_FR']['y']
        T_FRz=cfg['T_FR']['z']
        T_FRf=cfg['T_FR']['f']

        T_FLx=cfg['T_FL']['x']
        T_FLy=cfg['T_FL']['y']
        T_FLz=cfg['T_FL']['z']
        T_FLf=cfg['T_FL']['f']

        T_BRx=cfg['T_BR']['x']
        T_BRy=cfg['T_BR']['y']
        T_BRz=cfg['T_BR']['z']
        T_BRf=cfg['T_BR']['f']

        T_BLx=cfg['T_BL']['x']
        T_BLy=cfg['T_BL']['y']
        T_BLz=cfg['T_BL']['z']
        T_BLf=cfg['T_BL']['f']

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

        # input vector multiplier
        self.u_scale=np.array([T_Lf,T_Rf,T_Ff,T_Bf,T_FLf,T_FRf,T_BLf,T_BRf])

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

        ####################################################################
        #TODO#
        #Check if the xB and yB here are correct, bottom or back
        #Something might be wrong with the psi equation, yaw momement generated when strafing
        #looks like some anomalies result from displacement of bouancy from gravity
        ####################################################################
        

        #Drag Forces
        self.F_D_x = lambda x :  np.sign(x[1])*(self.rho/2)*(self.Cd_x*abs(cos(x[10])*cos(x[8]))*self.A_x + self.Cd_y*abs(sin(x[10]))*self.A_y + self.Cd_z*abs(sin(x[10]))*self.A_z)*(x[1]**2); 
        self.F_D_y = lambda x : np.sign(x[3])*(self.rho/2)*(self.Cd_x*abs(sin(x[10]))*self.A_x + self.Cd_y*abs(cos(x[10])*cos(x[6]))*self.A_y + self.Cd_z*abs(sin(x[6]))*self.A_z)*(x[3]**2); 
        self.F_D_z = lambda x : np.sign(x[5])*(self.rho/2)*(self.Cd_x*abs(sin(x[8]))*self.A_x + self.Cd_y*abs(sin(x[6]))*self.A_y + self.Cd_z*abs(cos(x[6])*cos(x[8]))*self.A_z)*(x[5]**2); 

        self.F_D_phi = lambda x : (self.rho/(2*self.Ixx))*( \
            self.Cd_zL*self.A_zL*self.dy_zL*(x[7]*self.dy_zL + x[1]*abs(sin(x[8])) + x[3]*abs(sin(x[6])) + x[5]*abs(cos(x[6])*cos(x[8]))) + \
            self.Cd_zR*self.A_zR*self.dy_zR*(x[7]*self.dy_zR + x[1]*abs(sin(x[8])) + x[3]*abs(sin(x[6])) + x[5]*abs(cos(x[8])*cos(x[6]))) + \
            self.Cd_yB*self.A_yB*self.dz_yB*(x[7]*self.dz_yB + x[1]*abs(sin(x[10])) + x[3]*abs(cos(x[10])*cos(x[6])) + x[5]*abs(sin(x[6]))) + \
            self.Cd_yT*self.A_yT*self.dz_yT*(x[7]*self.dz_yT + x[1]*abs(sin(x[10])) + x[3]*abs(cos(x[10])*cos(x[6])) + x[5]*abs(sin(x[6]))))
        
        self.F_D_theta = lambda x : (self.rho/(2*self.Iyy))*( \
            self.Cd_xB*self.A_xB*self.dz_xB*(x[9]*self.dz_xB + x[1]*abs(cos(x[10])*cos(x[8])) + x[3]*abs(sin(x[10])) + x[5]*abs(sin(x[8]))) + \
            self.Cd_xT*self.A_xT*self.dz_xT*(x[9]*self.dz_xT + x[1]*abs(cos(x[10])*cos(x[8])) + x[3]*abs(sin(x[10])) + x[5]*abs(sin(x[8]))) + \
            self.Cd_zF*self.A_zF*self.dx_zF*(x[9]*self.dx_zF + x[1]*abs(sin(x[8])) + x[3]*abs(sin(x[6])) + x[5]*abs(cos(x[6])*cos(x[8]))) + \
            self.Cd_zB*self.A_zB*self.dx_zB*(x[9]*self.dx_zB + x[1]*abs(sin(x[8])) + x[3]*abs(sin(x[6])) + x[5]*abs(cos(x[6])*cos(x[8]))))

        self.F_D_psi = lambda x : (self.rho/(2*self.Izz))*( \
            self.Cd_xR*self.A_xR*self.dy_xR*(x[11]*self.dy_xR + x[1]*abs(cos(x[10])*cos(x[8])) + x[3]*abs(sin(x[10])) + x[5]*abs(sin(x[8]))) + \
            self.Cd_xL*self.A_xL*self.dy_xL*(x[11]*self.dy_xL + x[1]*abs(cos(x[10])*cos(x[8])) + x[3]*abs(sin(x[10])) + x[5]*abs(sin(x[8]))) + \
            self.Cd_yF*self.A_yF*self.dx_yF*(x[11]*self.dx_yF + x[1]*abs(sin(x[10])) + x[3]*abs(cos(x[10])*cos(x[6])) + x[5]*abs(sin(x[6]))) + \
            self.Cd_yBa*self.A_yBa*self.dx_yBa*(x[11]*self.dx_yBa + x[1]*abs(sin(x[10])) + x[3]*abs(cos(x[10])*cos(x[6])) + x[5]*abs(sin(x[6]))))

        #State Derivatives
        self.xdotdot = lambda x,u : ((u[0]+u[1])*(cos(x[10]))*(cos(x[8])) + (u[2]+u[3])*(-sin(x[10])) + (u[4]+u[5]+u[6]+u[7])*(sin(x[8])) - self.F_D_x(x))/self.m; 
        self.ydotdot = lambda x,u : ((u[0]+u[1])*(sin(x[10])) + (u[2]+u[3])*(cos(x[10])*cos(x[6])) - (u[4]+u[5]+u[6]+u[7])*sin(x[6]) - self.F_D_y(x))/self.m; 
        self.zdotdot = lambda x,u : ((u[0]+u[1])*(-sin(x[8])) + (u[2]+u[3])*sin(x[6]) + (u[4]+u[5]+u[6]+u[7])*(cos(x[6])*cos(x[8])) - self.F_D_z(x) - self.Fb)/self.m + self.g; 

        self.phidotdot = lambda x,u : (1/self.Ixx)*((u[5]+u[7])*self.phi_R - (u[4]+u[6])*self.phi_L) - self.F_D_phi(x) \
        + (self.Fb/self.Ixx)*(self.phi_bz*cos(x[8])*sin(x[6]) - self.phi_by*(cos(x[8])*cos(x[6]))); 
        
        self.thetadotdot = lambda x,u : (1/self.Iyy)*(-(u[7]+u[6])*self.theta_B + (u[4]+u[5])*self.theta_F) - self.F_D_theta(x) \
        + (self.Fb/self.Iyy)*(self.theta_bx*cos(x[8])*cos(x[6]) + self.theta_bz*sin(x[8])); 
        
        self.psidotdot = lambda x,u : (1/self.Izz)*(u[0]*self.psi_L - u[1]*self.psi_R + u[2]*self.psi_F - u[3]*self.psi_B) - self.F_D_psi(x) \
        + (self.Fb/self.Izz)*(self.psi_bx*cos(x[8])*sin(x[6]) - self.psi_by*sin(x[8])); 



        #Convenience Collector
        self.accels = lambda x,u : [self.xdotdot(x,u),self.ydotdot(x,u),self.zdotdot(x,u),
                self.phidotdot(x,u),self.thetadotdot(x,u),self.psidotdot(x,u)]; 

    def makeNoise(self):
        #Define assumed Process and Measurement Noise Matrices    
        #NOTE: These are fake...
        self.R = np.identity(12)*.1;  
        self.Q = np.identity(10)*.1; 

    def kinematics(self,x,u):
        a = self.accels(x,u); 
        t = self.dt
        k = [0 for i in range(0,len(x))]; 
        for i in range(0,len(a)):
            k[i*2] = (x[i*2] + x[i*2+1]*t + .5*a[i]*t**2); 
            k[i*2+1] = (x[i*2+1] + a[i]*t**2); 

        return k; 


    def measurement(self,x,u):

        y = [0 for i in range(0,10)]; 
        a = self.accels(x,u);

        y[0:3] = a[0:3]; 
        y[3:8] = x[6:11]; 
        y[9] = x[2];  

        return y; 


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


    def update(self,mean,sigma,u,z):

        #Using the Thrun Page 70 algorithm
        #Note for when you debug this: The fact that means come in as row vectors 
        #is forcing the covariances to be non positive definite somehow


        #Confirm inputs in numpy formats
        if(not isinstance(mean,np.matrix)):
            x = np.matrix(mean).T; 
        else:
            x = mean; 

        if(not isinstance(sigma,np.matrix)):
            P = np.matrix(sigma); 
        else:
            P = sigma; 


        #Augment Matrices
        #xa = x.append()


        #set constants
        n = 12; 
        alpha = 1e-3;
        beta = 2; 
        kappa = 0; 
        lamb = (alpha**2)*(n+kappa) - n; 

        #get points
        points = []; 
        points.append(x.T); 
        

        modSig = sqrtm((n+lamb)*P); 
        for i in range(0,n):
            points.append(x.T + modSig[:,i]);
        for i in range(n,2*n):
            points.append(x.T - modSig[:,i-n]); 

        for i in range(0,len(points)):
            points[i] = points[i].T; 

        #set weights
        weights_m = []; 
        weights_c = []; 

        weights_m.append(lamb/(n+lamb)); 
        weights_c.append((lamb/(n+lamb)) + (1-alpha**2 + beta)); 

        for i in range(0,2*n):
            weights_m.append(1/(2*(n+lamb))); 
            weights_c.append(1/(2*(n+lamb))); 


        #print(weights_m);

        #print(weights_m)

        #print(sum(weights_m));
        #print(sum(weights_c));

        #mean_prop,sig_prop = self.prediction(chi); 
        chi = []; 
        for i in range(0,len(points)):
            chi.append(np.matrix(self.kinematics(points[i].T.tolist()[0],u)).T);  

        #print(len(chi))
        #print(sum(weights_m))


        #print(chi[1],weights_m[1]); 
        #print(chi[13],weights_m[13]); 



        #print(weights_m[0]*chi[0])


        mean_prop = np.matrix(np.zeros(shape=(len(x)))).T; 
        for i in range(0,len(chi)):
            #print(chi[i])
            mean_prop += weights_m[i]*chi[i]; 
            #print(sum(mean_prop))
        
        #print(mean_prop);

        #print(np.sum(chi)); 

        #print(mean_prop)


        sig_prop = np.zeros(shape=(len(x),len(x))); 
        for i in range(0,len(chi)):
            sig_prop += weights_c[i]*(chi[i]-mean_prop)*((chi[i]-mean_prop).T) + self.R; 

        

        #get new points
        pointsBar = []; 
        pointsBar.append(mean_prop.T); 

        modSigBar = sqrtm((n+lamb)*sig_prop); 


        for i in range(0,n):
            pointsBar.append(mean_prop.T + modSigBar[:,i])
        for i in range(n,2*n):
            pointsBar.append(mean_prop.T + modSigBar[:,i-n]); 

        for i in range(0,len(pointsBar)):
            pointsBar[i] = pointsBar[i].T; 


        #print(pointsBar[1])

        #project through meas
        gamma = []; 
        for i in range(0,len(pointsBar)):
            gamma.append(np.matrix(self.measurement(pointsBar[i].T.tolist()[0],u)).T); 



        #recombine points
        zhat = np.matrix(np.zeros(shape=(len(gamma[0])))).T; 
        for i in range(0,len(gamma)): 
            #print(weights_m[i])
            zhat += weights_m[i]*gamma[i];


        Pz = np.matrix(np.zeros(shape=(len(gamma[0]),len(gamma[0])))); 
        for i in range(0,len(gamma)):
            Pz += weights_c[i]*(gamma[i]-zhat)*((gamma[i]-zhat).T) + self.Q; 


        Px = np.matrix(np.zeros(shape=(len(x),len(gamma[0])))); 
        for i in range(0,len(gamma)):
            #print(chi[i].shape); 
            #print(gamma[i].shape)
            #print(chi[i]-mean_prop)
            Px += weights_c[i]*(chi[i]-mean_prop)*(gamma[i]-zhat).T; 

        #Px = Px.T; 
        #comput kalman gain
        K = Px*(Pz.I); 

        #print((K*np.matrix(z-zhat).T).shape)


        #get updated state estimate and covariance
        #x = mean_prop+(K*np.matrix(z-zhat).T).T; 
        newX = mean_prop + (K*(np.matrix(z).T-zhat)); 
        

        newP = sig_prop-K*Pz*K.T; 

        #print(det(P));

        #print(x)
        #print(x.tolist()[0])
        return newX,newP; 


class EKF(UKF):

    def __init__(self):
        print("Warning: EKF Not Relinearizing, do not use")
        UKF.__init__(self)
        self.controller = np.load('position_hold.npy').item(); 
        #print(self.controller.item().keys());
        self.G = np.matrix(self.controller['A']); 
        self.H = np.matrix(self.controller['C']); 

    def relinearize(self,x,u):

        dt = 1/25; 

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

        self.G = np.matrix(A); 
        self.H = np.matrix(C); 




    
    def ekfUpdate(self,mean,sigma,u,z):
        
        #relinearize
        self.relinearize(mean,u); 


        z = np.matrix(z).T;

        muBar = self.kinematics(mean,u); 

        mean = np.matrix(mean).T; 
        sigma = np.matrix(sigma);

        muBar = np.matrix(muBar).T; 

        sigBar = self.G*sigma*self.G.T + self.R; 
        K = sigBar*self.H.T*((self.H*sigBar*self.H.T + self.Q).I); 
        muNew = muBar + K*(z-np.matrix(self.measurement(muBar,u)).T); 
        sigNew = (np.identity(12)-K*self.H)*sigBar; 

        a = muNew.T.tolist()[0]; 
        
        for i in range(0,len(a)):
            muNew[i] = a[i].tolist()[0]; 
        #print(np.array(muNew).T.tolist()[0]); 
        muNew = np.array(muNew).T.tolist()[0]

        return muNew,sigNew; 

def nonLinearUnitTests():

    fil = UKF();
    
    print("Zero State, Thruster Input")

    #Stationary
    x = [0,0,0,0,0,0,0,0,0,0,0,0]; 
    u = [0,0,0,0,0,0,0,0]; 
    a = fil.accels(x,np.array(u)*fil.u_scale); 
    a = ['%.2f' % b for b in a]
    print("Zero Input:               {}".format(a)); 


    #Forward
    x = [0,0,0,0,0,0,0,0,0,0,0,0]; 
    u = [1,1,0,0,0,0,0,0]; 
    a = fil.accels(x,np.array(u)*fil.u_scale); 
    a = ['%.2f' % b for b in a]
    print("Forward Thrusters:        {}".format(a)); 

    #Strafe
    x = [0,0,0,0,0,0,0,0,0,0,0,0]; 
    u = [0,0,1,1,0,0,0,0]; 
    a = fil.accels(x,np.array(u)*fil.u_scale); 
    a = ['%.2f' % b for b in a]
    print("Strafe Right Thrusters:   {}".format(a));

    #Down
    x = [0,0,0,0,0,0,0,0,0,0,0,0]; 
    u = [0,0,0,0,.25,.25,.25,.25]; 
    a = fil.accels(x,np.array(u)*fil.u_scale); 
    a = ['%.2f' % b for b in a]
    print("Down Thrusters:           {}".format(a));

    #Down and Forward
    x = [0,0,0,0,0,0,0,0,0,0,0,0]; 
    u = [1,1,0,0,.25,.25,.25,.25]; 
    a = fil.accels(x,np.array(u)*fil.u_scale); 
    a = ['%.2f' % b for b in a]
    print("Depth Forward Thrusters:  {}".format(a));


    #Barrel-Roll
    x = [0,0,0,0,0,0,0,0,0,0,0,0]; 
    u = [1,1,0,0,0,.5,0,.5]; 
    a = fil.accels(x,np.array(u)*fil.u_scale); 
    a = ['%.2f' % b for b in a]
    print("Barrel Roll Thrusters:    {}".format(a));

    print(""); 

    print("Movement States, Depth Thruster Inputs")
    #Forward
    x = [0,-0.1,0,0,0,0,0,0,0,0,0,0]; 
    u = [0,0,0,0,.25,.25,.25,.25]; 
    #u = [0,0,0,0,0,0,0,0]; 
    a = fil.accels(x,np.array(u)*fil.u_scale); 
    a = ['%.2f' % b for b in a]
    print("Depth Forward:      {}".format(a)); 

    print("X Drag: {}".format(fil.F_D_x(x))); 
    print("Y Drag: {}".format(fil.F_D_y(x))); 
    print("Z Drag: {}".format(fil.F_D_z(x))); 

    print("Roll Drag: {}".format(fil.F_D_phi(x))); 
    print("Pitch Drag: {}".format(fil.F_D_theta(x))); 
    print("Yaw Drag: {}".format(fil.F_D_psi(x))); 


    #Strafe
    x = [0,0,0,0.1,0,0,0,0,0,0,0,0]; 
    u = [0,0,0,0,.25,.25,.25,.25]; 
    #u = [0,0,0,0,0,0,0,0]; 
    a = fil.accels(x,np.array(u)*fil.u_scale); 
    a = ['%.2f' % b for b in a]
    print("Depth Strafe:       {}".format(a)); 

    # print("Roll Drag: {}".format(fil.F_D_phi(x))); 
    # print("Pitch Drag: {}".format(fil.F_D_theta(x))); 
    # print("Yaw Drag: {}".format(fil.F_D_psi(x))); 


    #Roll
    x = [0,0,0,0,0,0,0,0.1,0,0,0,0]; 
    u = [0,0,0,0,.25,.25,.25,.25]; 
    a = fil.accels(x,np.array(u)*fil.u_scale); 
    a = ['%.2f' % b for b in a]
    print("Depth Roll:         {}".format(a)); 


def ukfTest():

    fil = UKF(); 
    mean = [0,0,0,0,0,0,0,0,0,0,0,0]; 
    u = [1,1,0,0,0,0,0,0]
    u = np.array(u)*fil.u_scale
    nextX = fil.kinematics(mean,u); 
    sigma = np.identity(len(mean))*10;
    meas = fil.measurement(nextX,u);
    #print(len(meas));  

    m,P = fil.update(mean,sigma,u,meas); 

    print(m);  

def kinematicTest():
    fil = UKF(); 
    x = [0,0,0,0,0,0,0,0,0,0,0,0]; 
    u = [1,1,0,0,-1.2816,-1.2816,-1.2816,-1.2816]; 
    #a = fil.accels(x,np.array(u)*fil.u_scale); 
    #a = ['%.2f' % b for b in a]
    #print(a);
    #xnew = integrate.quad()


    # xnew = fil.kinematics(x,np.array(u)*fil.u_scale);
    # print(x); 
    # xnew = ['%.8f' % b for b in xnew]
    # print(xnew); 

    runs = 1; 
    #res = np.zeros(shape=(runs,12));  

    start = time.clock(); 
    for i in range(0,runs):
        #a = fil.accels(x,np.array(u)*fil.u_scale); 
        #a = fil.accels(x,np.array(u)*fil.u_scale); 
        #print('Accels:' + str(['%.2E' % b for b in a])); 
        x = fil.kinematics(x,np.array(u)*fil.u_scale);
        # y = fil.measurement(x,np.array(u)*fil.u_scale); 
        # print('Meas:  ' + str(['%.2E' % b for b in y])); 
        print('State: ' + str(['%.5f' % b for b in x])); 
        #print(['%.2f' % b for b in x]); 
        #res[i,:] = x; 

        # plt.plot(res[0],color='blue',label='X Distance'); 
        # plt.plot(res[1],color='green', label='X Velocity'); 
        # plt.axhline(a[0], color='red', label = 'X Accel'); 
        # plt.ylim([0,1.6])
        # plt.legend(); 
        # plt.pause(0.01); 
        # plt.cla(); 
        # plt.clf(); 
    delTime = time.clock()-start; 
    print("Loop time for {} runs: {:0.2f} seconds".format(runs,delTime)); 
    print("Time per loop: {:0.5f} seconds".format(delTime/runs))



def ekfTest():
    fil = EKF(); 
    mean = [0,0,0,0,0,0,0,0,0,0,0,0]; 
    u = [1,1,0,0,0,0,0,0]
    u = np.array(u)*fil.u_scale
    nextX = fil.kinematics(mean,u); 
    sigma = np.identity(len(mean))*0.0001;
    meas = fil.measurement(nextX,u);
    #print(len(meas));  

    m,P = fil.ekfUpdate(mean,sigma,u,meas); 

    #print(m)

def allFilterTest():
    filU = UKF(); 
    filE = EKF(); 

    mean = [0,0,0,0,0,0,0,0,0,0,0,0]; 
    u = [1000,1000,0,0,10000,0,0,0]
    #u = [0,0,0,0,0,0,0,0]; 
    sigma = np.matrix(np.identity(len(mean))*1);


    truePos = []; 
    truePos.append(mean); 



    uPos = []; 
    uSig = []; 
    ePos = []; 
    eSig = []; 
    uPos.append(mean); 
    ePos.append(mean); 
    uSig.append(sigma); 
    eSig.append(sigma); 

    for i in range(0,1000):
        print("Step: {}".format(i)); 
        nextX = filU.kinematics(truePos[-1],u); 
        meas = filU.measurement(nextX,u); 

        #umean,usig = filU.update(uPos[-1],uSig[-1],u,meas); 
        emean,esig = filE.ekfUpdate(ePos[-1],eSig[-1],u,meas); 


        truePos.append(nextX)
        #uPos.append(umean); 
        #uSig.append(usig); 
        ePos.append(emean); 
        eSig.append(esig); 

    fig,axarr = plt.subplots(6,2,sharex=True); 

    #print(truePos)
    for stateMonitor in range(0,6):
        for der in range(0,2):
            #stateMonitor = 4; 
            #uz = [a[stateMonitor*2+der] for a in uPos]; 

            tz = [a[stateMonitor*2+der] for a in truePos]; 
            ez = [a[stateMonitor*2+der] for a in ePos]; 

            error = [ez[i]-tz[i] for i in range(0,len(ez))]; 

            #print(eSig[0]); 
            es = [a.tolist()[stateMonitor*2+der][stateMonitor*2+der] for a in eSig]; 
            upperEs = [ez[i]+2*np.sqrt(es[i]) for i in range(0,len(ez))]; 
            lowerEs = [ez[i]-2*np.sqrt(es[i]) for i in range(0,len(ez))]; 

            # us = [a.tolist()[stateMonitor*2+der][stateMonitor*2+der] for a in uSig]; 
            # upperUs = [uz[i]+2*np.sqrt(us[i]) for i in range(0,len(uz))]; 
            # lowerUs = [uz[i]-2*np.sqrt(us[i]) for i in range(0,len(uz))]; 


            #print(es);

            x = [i for i in range(0,len(tz))]; 

            #axarr[stateMonitor][der].plot(x,tz,c='k',linestyle='-',linewidth=5,label='Truth Z'); 
            # axarr[stateMonitor][der].plot(x,uz,c='r',linestyle='-',linewidth=3,label='UKF Z'); 
            # axarr[stateMonitor][der].plot(x,upperUs,c='r',linestyle='--',linewidth=1); 
            # axarr[stateMonitor][der].plot(x,lowerUs,c='r',linestyle='--',linewidth=1); 
            #axarr[stateMonitor][der].fill_between(x,upperUs,lowerUs,alpha=0.25); 

            axarr[stateMonitor][der].plot(x,ez,c='b',linestyle='--',linewidth=3,label='EKF Z'); 
            axarr[stateMonitor][der].plot(x,upperEs,c='b',linestyle='--',linewidth=1); 
            axarr[stateMonitor][der].plot(x,lowerEs,c='b',linestyle='--',linewidth=1); 
            axarr[stateMonitor][der].fill_between(x,upperEs,lowerEs,alpha=0.25); 

            #plt.legend(); 
            #plt.show(); 
    plt.show();

if __name__ == '__main__':
    #nonLinearUnitTests(); 
    #ukfTest();
    #kinematicTest();  
    #ekfTest(); 
    
    allFilterTest(); 
