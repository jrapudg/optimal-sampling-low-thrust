using ControlSystems
using ForwardDiff
using DelimitedFiles
using LinearAlgebra
using PlotlyJS

earth = (
    μ = 3.986004418e14,
    G = 6.67430e-11,
    M = 5.9722e24,
    R = 6378.0e3,
    R_GEO = 42164000.0,
    R_LEO_ub = 8413.0e3
)

#ssma = 8413.0e3

ssma = 2000000.0 #keeping it the same as LEO_MAX from the dynamics in astrodynamics.cpp

m = 100

#Timestep
dt = 0.1

#number of states
nx = 6

u0 = [0,0,0]

#path of the data. //change for your path
#R=1 path
#data_path = "/home/fausto/optimal-sampling-low-thrust/orbital_planner/src/rrt_star/Output_R1_med.txt"
#R=10 path
#data_path = "/home/fausto/optimal-sampling-low-thrust/orbital_planner/src/rrt_star/Output_R10_med_new.txt"
#R=100 path
data_path = "/home/fausto/optimal-sampling-low-thrust/orbital_planner/src/rrt_star/Output_R100_med.txt"

reference_traj = readdlm(data_path, ',')

#truncate in case the file has extra lines
reference_traj = reference_traj[:,1:nx]

iters = 10

#spacecraft nonlinear relative dynamics model
function nonlinear_relative_keplerian_dynamics(X::Vector, F::Vector)
    x = X[1]
    y = X[2]
    z = X[3]
    ẋ = X[4]
    ẏ = X[5]
    ż = X[6]
    rt = ssma
    rtdot = sqrt(earth.μ/rt)
    rc = sqrt((rt + x)^2 + y^2 + z^2)
    θdot = sqrt(earth.μ/rt^3)
    ẍ = - (earth.μ / rc^3) * ( rt + x ) +  θdot^2 * x + 2 * θdot * (ẏ - y * (rtdot/rt)) + earth.μ / rt^2 + F[1] / m
    ÿ = - (earth.μ / rc^3) * y + θdot^2 * y - 2 * θdot * (ẋ - x * (rtdot/rt)) + F[2] / m
    z̈ = - (earth.μ / rc^3) * z + F[3] / m
    return [ẋ, ẏ, ż, ẍ, ÿ, z̈]

end

#RK4 integrator
function spacecraft_RK4(x,u)

    f1 = nonlinear_relative_keplerian_dynamics(x,u)
    f2 = nonlinear_relative_keplerian_dynamics(x + 0.5 *dt* f1, u)
    f3 = nonlinear_relative_keplerian_dynamics(x + 0.5 *dt* f2, u)
    f4 = nonlinear_relative_keplerian_dynamics(x + dt * f3, u)

    xn = x + (dt/6) * (f1 + 2*f2 + 2*f3 + f4)

    return xn

end

function run_lqr_twostates(initial_state, k)

    traj = zeros((6,10))
    controls= zeros((3,10))
    #testing with just one point
    #initial_state = reference_traj[k,:]
    next_state = reference_traj[k+1,:]

    #Convert to Float 64 Vectors
    #initial_state_f::Vector{Float64} = map(x -> float(x), initial_state)
    next_state_f::Vector{Float64} = map(x -> float(x), next_state)

    traj[:,1] = initial_state_f

    #linearize the dynamics at the next state (linearizing at zero control )
    A = ForwardDiff.jacobian(dx-> nonlinear_relative_keplerian_dynamics(dx, u0), next_state_f)
    B = ForwardDiff.jacobian(du-> nonlinear_relative_keplerian_dynamics(next_state_f, du), u0)

    #Discretize the dynamics
    Nx = 6
    Nu = 3

    #tuning these for the tracking controller
    #works with 1e4


    #working with good argument
    #Q = 10*Matrix(I,Nx,Nx)
    #R = 10*Matrix(I,Nu,Nu)

    Q = 1e5*Matrix(I,Nx,Nx)
    R = 1*Matrix(I,Nu,Nu)

    #change from continous to Discrete
    H = exp(dt*[A B; zeros(Nu, Nx+Nu)])

    Ad = H[1:Nx, 1:Nx]
    Bd = H[1:Nx, (Nx+1):end]

    K = dlqr(Ad, Bd, Q, R)

    #this is the controllability Matrix
    C = Bd
    for k = 1:Nx-1
        C = [C Ad*C[:,end-(Nu-1):end]]
    end

    #println("RANK OF C: ", rank(C))

    #initial control law
    u = -K*(initial_state_f - next_state_f)

    controls[:,1] = u

    #println("THIS IS initial u: ", u)

    for k in 1:iters-1
        
        #keep on updating the initial state
        traj[:,k+1] = spacecraft_RK4(traj[:,k], u)

        #updated control law since we update our position

        u = -K*(traj[:,k+1] - next_state_f)
        controls[:,k+1] = u

        #println("THIS IS U: ", u)

        #print("Residual ",k ,":",  norm(next_state_f - traj[:,k]), "\n")

        if norm(traj[:,k]- next_state_f) < 0.15
            #print("REACHED THE STATE")
            return traj, next_state_f, controls
        end
    end

    return traj, next_state_f, controls

end

#initial state
initial_state = reference_traj[1,:]
initial_state_f::Vector{Float64} = map(x -> float(x), initial_state)

#traj, next_state_f = run_lqr_twostates(initial_state, 1)

all_traj = []
all_controls = []

#append!(all_traj, ones(6))

for k in 1:size(reference_traj)[1]-1
#for k in 1:5

    traj, next_state_f, controls = run_lqr_twostates(initial_state_f, k)

    for i in 1:size(traj)[2]
        if sum(traj[:,i]) == 0
            break
        end
        append!(all_traj, traj[:,i])

    end

    for j in 1:size(controls)[2]
        if sum(controls[:,j]) == 0
            break
        end
        append!(all_controls, controls[:,j])

    end

    initial_state_f = all_traj[end-5:end] 

    #println("this is initial state: ", initial_state_f)

end


#the all traj, reshape
all_traj = reshape(all_traj, (6, size(all_traj)[1] ÷ 6))

#the all controls, reshape
all_controls = reshape(all_controls, (3, size(all_controls)[1] ÷ 3))

ref = PlotlyJS.scatter3d(x=reference_traj[:,1], y=reference_traj[:,2], z=reference_traj[:,3], mode="lines", name="Reference Planner Trajectory", line=attr(width=5))
lqr_traj = PlotlyJS.scatter3d(x=all_traj[1,:], y=all_traj[2,:], z=all_traj[3,:], mode="lines", name="TVLQR Trajectory", line=attr(width=5))
layout = Layout(title="LQR Trajectory", xaxis_title="X", yaxis_title="Y", zaxis_title="Z", legend=true)
PlotlyJS.plot([lqr_traj, ref], layout)

all_controls
#sum of the controls
sum_controls = sum(abs.(all_controls), dims=1)

#save sum_controls to a txt file. 
writedlm("sum_controls_R100.txt", sum_controls, ',')


#total fuel
total_fuel = sum(sum_controls)


sum_controls_R1 = sum_controls


#read the txt file and save into Matrix
sum_controls_R1 = readdlm("sum_controls_R1.txt", ',')
sum_controls_R10 = readdlm("sum_controls_R10.txt", ',')
sum_controls_R100 = readdlm("sum_controls_R100.txt", ',')

#plot the sum controls of all the R's saved in the text files
sum_controls_R1 = PlotlyJS.scatter(x=1:size(sum_controls_R1)[2], y=sum_controls_R1[1,:], mode="lines", name="Control R1", line=attr(width=5), color="blue")
sum_controls_R10 = PlotlyJS.scatter(x=1:size(sum_controls_R10)[2], y=sum_controls_R10[1,:], mode="lines", name="Control R10", line=attr(width=5), color="red")
sum_controls_R100 = PlotlyJS.scatter(x=1:size(sum_controls_R100)[2], y=sum_controls_R100[1,:], mode="lines", name="Control R100", line=attr(width=5), color="green")

layout = Layout(title="Sum of Controls", xaxis_title="Time", yaxis_title="Control Magnitude per Timestep", legend=true)
PlotlyJS.plot([sum_controls_R1, sum_controls_R10, sum_controls_R100], layout)


# all_controls_x = PlotlyJS.scatter(x=1:size(all_controls)[2], y=all_controls[1,:], mode="lines", name="Control X", line=attr(width=5), color="blue")
# all_controls_y = PlotlyJS.scatter(x=1:size(all_controls)[2], y=all_controls[2,:], mode="lines", name="Control Y", line=attr(width=5), color="red")
# all_controls_z = PlotlyJS.scatter(x=1:size(all_controls)[2], y=all_controls[3,:], mode="lines", name="Control Z", line=attr(width=5), color="green")

# layout = Layout(title="Controls", xaxis_title="Time", yaxis_title="Control Magnitude per Timestep", legend=true)
# PlotlyJS.plot([all_controls_x, all_controls_y, all_controls_z], layout)

#tests two subsequent states
#using PlotlyJS
# lqr_steer = PlotlyJS.scatter3d(x=traj[1,:], y=traj[2,:], z=traj[3,:], mode="lines", name="Spacecraft Trajectory", line=attr(width=5))
# initial = PlotlyJS.scatter3d(x=[traj[1,1]], y=[traj[2,1]], z=[traj[3,1]], mode="markers", name="Start", marker=attr(size=8))
# final = PlotlyJS.scatter3d(x=[traj[1,end]], y=[traj[2,end]], z=[traj[3,end]], mode="markers", name="End", marker=attr(size=8))
# desired_state = PlotlyJS.scatter3d(x=[next_state_f[1]], y=[next_state_f[2]], z=[next_state_f[3]], mode="markers", name="Desired", marker=attr(size=8))
# layout = Layout(title="LQR_Steer", xaxis_title="X", yaxis_title="Y", zaxis_title="Z", legend=true)
# PlotlyJS.plot([lqr_steer, initial, final, desired_state], layout)


