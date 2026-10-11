import numpy as np 
import matplotlib.pyplot as plt 
import math 
plt.style.use('dark_background')

## State vector : X = [x y phi v] ## phi = yaw angle , v = velocity

def control_vector(time): ## [v,w] where v is the constant velocity and w is the yaw rate 
    ## v = 1 ## constant value 
    if time >= 40.0:
        w = 0.1
    elif time >= 35.0:   ## dt is the function of time here , it is changing according to this equation 
        w = -0.2
    elif time >= 25.0:
        w = 0.3
    elif time >= 15.0:
        w = -0.1
    else:
        w = 0.1
    return np.array([[1],[w]])  ## velocity v = 1

def state_transition_model(state,control,dt):
    ## x_t = g(control , state) 
    ## x_t = G1 . state + G2 . control 
    G1 = np.identity(len(state))
    G2 = np.array([
        [dt*math.cos(state[2,0]) , 0],
        [dt*math.sin(state[2,0]) , 0],
        [0,dt],
        [0,0]
    ])
    next_state = G1 @ state + G2 @ control 
    return next_state 

def observation_model(state):
    ## z_t = h(x_t) or h(state)
    ## z_t = H1 . state 
    H1 = np.array([[1,0,0,0],[0,1,0,0]])
    observe_at_state_t = H1 @ state 
    return observe_at_state_t 

def actual_measure_with_noise(actual_state,belief_state,control,dt):
    actual_state = state_transition_model(actual_state , control , dt) ## This gives the next true state withput any nosie 
    ## Control vector noise : covariance matrix : square is done to convert the std_deviation to variance 
    control_noise = np.diag([1 , np.deg2rad(25)])**2 
    ## Observation noise : covariance matrix 
    sensor_observe_noise  = np.diag([0.5 , 0.5 ])**2 
    ## First step is to move from one state vector to next state vector and finding belief of where you are
    ## Our next belief state will be computed through previous belief state 
    # adding noise to our control vector because this will be used in computing the next belief state in state_transition_model 
    control_vec_with_noise = control + control_noise @ np.random.randn(2,1) 
    belief_state = state_transition_model(belief_state,control_vec_with_noise,dt)
    ## Once we reached to next belief state, we will observe through our sensor where we actually are 
    ## that observation will also have noise , ### Here we are observing with true_state values 
    observation_with_noise = observation_model(actual_state) + (sensor_observe_noise @ np.random.randn(2,1))
    return actual_state , belief_state , observation_with_noise ## (x_true , x_belief , z)

def jacobian_STM(state,control,dt): ## jacobian matrix of state transition model 
        # [[da/dx , da/dy , da/d(phi) , da/dv]
        # [da/dx , da/dy , da/d(phi) , da/dv]
        # [da/dx , da/dy , da/d(phi) , da/dv]
        # [da/dx , da/dy , da/d(phi) , da/dv]]
    ## which jacobian to pick : current belief state which is computed in "actual_measure_with_noise" function
    ## Actual jacobian [1 0 -dt*v*sin(phi) dt*cos(phi)]
                    #  [0 1 dt*v*cos(phi) dt*sin(phi)]
                    #  [0 0 1 0]
                    #  [0 0 0 1]
    phi = state[2,0]
    v = control[0,0]
    jacob_g = np.array([
        [1.0 , 0.0 , -dt*v*math.sin(phi) , dt*math.cos(phi)],
        [0.0 , 1.0 , dt*v*math.cos(phi) , dt*math.sin(phi)],
        [0.0,0.0,1.0,0.0],
        [0.0,0.0,0.0,1.0]
    ])
    return jacob_g 

def jacobian_OM(state , dt): ## Jacobian matrix of the observation model 
    # [[da/dx , da/dy , da/d(phi) , da/dv]
    #  [da/dx , da/dy , da/d(phi) , da/dv] ] 
    ## 4x2 matrix only becasue obsevation is 2x1 matrix only 
    ## Although it depends on the current belief state vector and dt , but in our case it will remain constant 
    jacob_h = np.array([
        [1,0,0,0], ## da/dx will be 1 , rest are zero
        [0,1,0,0]  ## da/dy is 1 ,rest are zero 
    ])
    return jacob_h

def extended_kalman_filter(state,gauss_covariance,control,observe,dt):
    ## parameters : state : belief state from "actual_measure_with_noise" 
    #               gauss_covariance : input assumption needed
    #               control : control_vector with noise obtained from "actual_measure_with_noise"
    #               observe : observation_with_noise obtained from "actual_measure_with_noise"

    ## We need : 1) Gaussian nosie for state transition model 2) gaussian nosie for observation model
    R_gauss_STM = np.diag([0.1,0.1,np.deg2rad(1.0),1.0])**2 ## Gaussian nosie for state transition model
    Q_gauss_OM  = np.diag([1.0 , 1.0])**2

    ## Steps in Extended Kalman filter :
    ## Step 1 : compute prior belief state 
    prior_state = state_transition_model(state , control , dt)
    ## step 2 : compute the prior gauss covariance matrix 
    jacob_g = jacobian_STM(state,control,dt) ## Intermediate step 
    prior_gauss_covariance = (jacob_g @ gauss_covariance @ jacob_g.T) + R_gauss_STM 
    ## step 3 : compute the Kalman gain
    jacob_h = jacobian_OM(state , dt)  ## intermediate step
    inverse_value = np.linalg.inv((jacob_h @ prior_gauss_covariance @ jacob_h.T) + Q_gauss_OM) ## intermediate step
    kalman_gain = prior_gauss_covariance @ jacob_h.T @ inverse_value
    ## step 4 : compute the next belief state 
    observe_difference = observe - observation_model(prior_state) ## intermediate step
    state = prior_state + (kalman_gain @ observe_difference)
    ## Step 5 : compute the next gauss_covariance 
    identity_matrix = np.identity(len(state))  ## inntermediate step
    gauss_covariance = (identity_matrix - (kalman_gain @ jacob_h)) @ prior_gauss_covariance  
    return state , gauss_covariance   


def main():
    true_state = np.zeros((4,1)) ## initial state vector --> transpose of [0,0,0,0] 
    belief_state = np.zeros((4,1))  ## our initial belief of state vector--> transpose of [0,0,0,0] 
    observe = np.zeros((2,1)) ## our observation of x and y values when we are at our belief_state--> transpose of [0,0]
    gauss_covariance = np.eye(len(belief_state))
    ## storing the historical data for true_stae,belief_state and observation
    hist_true_state = true_state
    hist_belief_state = belief_state 
    hist_observe = observe 

    simulation_time = 80 
    dt = 0.02 
    time = 0.0

    while time <= 50:
        time += dt 
        ctrl_vec = control_vector(time)
        ## Compute true state,belief state with noise,observation_with_noise
        ## Below belief state is the current belief state
        true_state,belief_state,observe = actual_measure_with_noise(true_state,belief_state,ctrl_vec,dt)
        ## Compute next belief state and its corresponding next gauss_covariance
        ## Here belief state computed in Extended Kalman filter is the next belief state 
        belief_state , gauss_covariance = extended_kalman_filter(belief_state,gauss_covariance,ctrl_vec,observe,dt)

        ## Storing the true_state, belief_state , observation data in the hist variable 
        hist_true_state = np.hstack((hist_true_state , true_state))
        hist_belief_state = np.hstack((hist_belief_state , belief_state))
        hist_observe = np.hstack((hist_observe , observe))

        plt.cla()
        plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(hist_observe[0,:] , hist_observe[1,:] , ".g")
        plt.plot(hist_true_state[0,:] , hist_true_state[1,:] , "-y")
        plt.plot(hist_belief_state[0,:] , hist_belief_state[1,:] , "-r")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)

        ## Printing the data simultaneously 
        print(f"x_true : {true_state[0,0]} x_belief : {belief_state[0,0]} , y_true : {true_state[1,0]} , y_belief : {belief_state[1,0]}")
        print(f"velocity : {belief_state[3,0]} , yaw_angle : {belief_state[2,0]} , x_observe : {observe[0,0]} , y_observe : {observe[1,0]}")
        

if __name__ == '__main__':
    main()








#








