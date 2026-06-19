from core.entities import Propulsion
import mission_design.orbit_transfers as hm
import mission_design.hyp_capture as hp

# Complete Hohmann Transfer Impulse Set up     
def hohmann_transfer(r1, r2, mu, T0=0.001):
    
    dV1, dV2, T_trans = hm.circular_HT(r1, r2, mu)
    _, T2 = hm.T_hohmann(r1, r2, mu)
    
    T_tot = T0 + T_trans/2 + T2
    impulse0 = Propulsion(tf=T0,             dVx=0.0, dVy=dV1, dVz=0.0)
    impulse1 = Propulsion(tf=T0 + T_trans/2, dVx=0.0, dVy=dV2, dVz=0.0)
    
    return [impulse0, impulse1], T_tot

# Complete Elliptical Hohmann Transfer Impulse Set up 
def elliptic_hohmann_transfer(r1, a1, r2, a2, mu, T0=0.001):

    dV1, dV2, T_trans = hm.elliptic_HT(r1, a1, r2, a2, mu)
    _, T2 = hm.T_hohmann(a1, a2, mu)

    T_tot = T0 + T_trans/2 + T2
    impulse0 = Propulsion(tf=T0,             dVx=0.0, dVy=dV1, dVz=0.0)
    impulse1 = Propulsion(tf=T0 + T_trans/2, dVx=0.0, dVy=dV2, dVz=0.0)
    
    return [impulse0, impulse1], T_tot

# Complete Bielliptical Hohmann Transfer Impulse Set up 
def bielliptic_transfer(Ra, Rb, Rc, mu, T0=0.001):

    dV1, dV2, T_t1, T_t2 = hm.bi_elliptic(Ra, Rb, Rc, mu)
    _, T2 = hm.T_biell(Ra, Rb, Rc, mu)
    T_tot = T0 + T_t1/2 + T_t2/2 + T2

    impulse0 = Propulsion(tf=T0,          dVx=0.0, dVy=dV1, dVz=0.0)
    impulse1 = Propulsion(tf=T0 + T_t1/2, dVx=0.0, dVy=dV2, dVz=0.0)
    
    return [impulse0, impulse1], T_tot

# Hyperbolic Insertion Impulse Set up 
def hyperbolic_capture(R0, V0, mu, T0=0):

    dV = hp.dV_hyper(R0, V0, mu)
    T = hp.T_hyper(T0, R0, V0, mu)
    
    impulse0 = Propulsion(tf=T, dVx=-dV[0], dVy=dV[1], dVz=dV[2])
    
    return impulse0, T