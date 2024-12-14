import numpy as np
from scipy.integrate import solve_ivp
import control as ct
import matplotlib.pyplot as plt
import matplotlib.animation as animation


"""
  COEN 5830 - Intro to Robotics
  Final Project: Inverted pendulum control
  Student(s): Michael Bernabei, Ian Mcconachie                                                                       
  Fall 2024
  Instructor: Dr. Leopold Beuken
                                                                                                                                                                
  Program description:  Developes a controller for an inverted pendulum

  Author(s): Michael Bernabei, Ian Mcconachie
  Date: 11/13/2024

"""

# this builds the gain constant matrix K such that u = -K*x
def LQR(k_pos,k_theta,cost):
    
    model_deviation = 0.05
    M = 1 + model_deviation; 
    m = 0.5 - model_deviation; 
    l = 0.4 + model_deviation; 
    b = 0.01 + model_deviation*10; # hard to measure so make a guess
    g = 9.81;

    qx = k_pos * 100
    qt = k_theta * 100

    A = np.array([[0,1,0,0],[0,-b/M , -m*g/M , 0],[ 0 ,0 ,0 , 1 ] , [ 0 , b/(M*l) , (M+m)*g/(M*l) , 0]]);
    B = np.array([[0] , [1/M] , [0] , [-1/(M*l)]])

    Q = np.array([[qx , 0 ,0 ,0 ] , [ 0 , 0 ,0, 0 ] , [ 0,0,qt,0 ] , [0,0,0,0]])  
    R = cost;  

    K, S, E = ct.lqr(A, B, Q, R)
    return K

# This function takes the state and time and gain matrix and outputs the force applied 
def state_controller(t,X,K): 
   
    F_limit = 40.0 #N
    
    def clamp(value, bound):
        return max(-bound, min(value, bound))
    
    X_array = np.array([X]).T 
    X_prime = -np.array([[ -1] , [0] , [0] , [0]]) + X_array
    
    # set point scheduling 
    F = float(-np.dot(K,X_prime))
    if t > 1.5:
        F = float(-np.dot(K,X_array))
    if t > 4:
        F = float(-np.dot(K,X_prime))
    
    F = clamp(F,F_limit)
    
    return F

# this is the implementation of the non-linear equations of motion 
def EOM(t, X ):
    # X = [ X , X' , Theta , Theta']^T
    
    x = X[0];
    dx = X[1];
    T = X[2];
    dT = X[3]
    
    F = state_controller(t,X,K)
    
    g = 9.81 #m/s^2
    
    M = 1 # Kg
    m = 0.5 # Kg
    l = 0.4 # m
    b = 0.01 # Ns/m
   
    dX1 = X[1]                                                                                 
    dX1_dot = ( m*l*(dT*dT)*np.sin(T) - g*m*np.cos(T)*np.sin(T) - b*dx + F ) / ( (M+m) - m*np.cos(T)*np.cos(T) )
    dX2 = X[3]
    dX2_dot = ( (M+m)*g*np.sin(T) - m*l*(dT*dT)*np.cos(T)*np.sin(T) + b*dx*np.cos(T) - F*np.cos(T) ) / (  l*(M+m) - m*l*np.cos(T)*np.cos(T) )
    
    return [dX1, dX1_dot , dX2 , dX2_dot]

# This function solves the EOM 
def simulate_sys_with_control(EOM,t_span,X0):
 
    solution = solve_ivp( EOM, (0,t_span) , X0 , method='RK45',max_step=0.02)
    return solution
 
# this function plots the response of the states over time for the pendulum
def plot_info(solution):
    
    t = solve.t
    x = solve.y[0]
    dx = solve.y[1]
    theta = solve.y[2]
    dTheta =  solve.y[3]
    X = np.array([x,dx,theta,dTheta]).T

    t_set = np.linspace(0, 6.5, 1000)
    theta_set = np.zeros(len(t_set))
    pos_set = np.where((t_set > 1.5) & (t_set <= 4), 0, -1)
    
    n = len(solve.y[0])
    F = np.zeros(n)
    for i in range(n):
        F[i] = state_controller(t[i],X[i],K)
    
    plt.figure(figsize=(12,10))

    plt.subplot(311)
    plt.title('LQR Feedback Controller State Response')
    plt.plot(t,F,'m',lw=2, label='Applied Force ( F )')
    plt.legend(loc=1)
    plt.ylabel('Force (N)')
    plt.xlim(t[0],t[-1])
    plt.grid(True)

    plt.subplot(312)
    plt.plot(t_set,pos_set , 'g--',lw=1.5, label='Set Point')
    plt.plot(t,x,'r',lw=2, label='Cart Position ( x )')
    plt.legend(loc=1)
    plt.ylabel('Position (m)')
    plt.xlim(t[0],t[-1])
    plt.grid(True)

    plt.subplot(313)
    plt.plot(t_set, theta_set, 'g--',lw=1.5, label='Set Point')
    plt.plot(t,-theta*180/np.pi,'b',lw=2 , label='Rod Angle ( θ )')
    plt.legend(loc=1)
    plt.ylabel('Angle (°)')
    plt.xlabel('Time (s)')
    plt.xlim(t[0],t[-1])
    plt.grid(True)

# This function plots the animation of the pendulum
def plot_animation(solution, dark): 
    if dark:
        col_background_1 = 'w'
        col_background_2 = 'gray'
        plt.style.use('dark_background')
        
    else:
        col_background_1 = 'k'
        col_background_2 = 'k'
    
    t = solve.t
    x = solve.y[0]
    dx = solve.y[1]
    theta = solve.y[2]
    dTheta =  solve.y[3]
    X = np.array([x,dx,theta,dTheta]).T
    
    bound_pos = 0.8
    bound_neg = 1.8

    n = len(solve.y[0])
    F = np.zeros(n)
    for i in range(n):
        F[i] = state_controller(t[i],X[i],K)
  
    F_scaler = F / max(F) 
    
    def init():
        mass1.set_data([],[])
        mass2.set_data([],[])
        line.set_data([],[])
        F_arrow.set_data([],[])
        time_text.set_text('')
        return line, mass1, mass2, F_arrow , time_text

    def animate(i):
        mass1.set_data([x1[i]],[y1[i]-0.1])
        mass2.set_data([x2b[i]],[y2b[i]])
        line.set_data([x1[i],x2[i]],[y1[i],y2[i]])
        F_arrow.set_data([x1[i],x1[i]+F_scaler[i]],[-0.1,-0.1])
        time_text.set_text(time_template % t[i])
        return line, mass1, mass2, F_arrow , time_text
    
    x1 = x
    y1 = np.zeros(len(t))

    #suppose that l = 1
    col = '#D4AF37'
    l = 0.4
    x2 = l*np.sin(theta)+x1
    x2b = (l+0.05)*np.sin(theta)+x1
    y2 = l*np.cos(theta)-y1
    y2b = (l+0.05)*np.cos(theta)-y1

    fig = plt.figure(figsize=(8,6.4))
    ax = fig.add_subplot(111,autoscale_on=False,xlim=(-bound_neg,bound_pos),ylim=(-0.4,1.2))
    ax.set_xlabel('position ( m )')
    ax.get_yaxis().set_visible(False)

    crane_rail, = ax.plot([-(bound_neg+.5),bound_pos + 0.5],[-0.2,-0.2], color= col_background_1 ,lw=4)
    start, = ax.plot([-1,-1],[-1.5,1.5],':', color= col_background_1,lw=1.5)
    objective, = ax.plot([0,0],[-0.5,1.5],':', color= col_background_1,lw=1.5)

    mass1, = ax.plot([],[],linestyle='None',marker='s',\
                    markersize=44,markeredgecolor=col_background_2,\
                    color=col,markeredgewidth=2)

    mass2, = ax.plot([],[],linestyle='None',marker='o',\
                    markersize=20,markeredgecolor=col_background_2,\
                    color=col,markeredgewidth=2)

    line, = ax.plot([],[],'o-',color=col,lw=4,\
                    markersize=6,markeredgecolor=col_background_2,\
                    markerfacecolor=col_background_2)
    
    F_arrow, = ax.plot([],[],color='r',lw=2,marker='D',markersize=6,markeredgecolor='r',markerfacecolor='k')

    time_template = 'time = %.1fs'
    time_text = ax.text(0.05,0.9,'',transform=ax.transAxes)
    start_text = ax.text(-1.06,-0.3,'start',ha='right')
    end_text = ax.text(0.06,-0.3,'objective',ha='left')

    plot = animation.FuncAnimation(fig, animate,np.arange(1,len(t)), interval=40,blit=False,init_func=init)
    
    plt.show()


#### This sets up the environment 

##### time span and initial conditions
# X = [ X , X' , Theta , Theta']^T
t_span = 6.5 # sec
X0 = [-1 , 0 , 20*np.pi/180 , 0] 

# gains to balance the system response 
position_gain = 15
angle_gain = 12
action_gain = 0.1

dark_theme = False

K = LQR( position_gain , angle_gain , action_gain ) # solve for the control gain using LQR
# print(K)
solve = simulate_sys_with_control(EOM,t_span,X0)
p1 = plot_info(solve)
p2 = plot_animation(solve,dark_theme)
