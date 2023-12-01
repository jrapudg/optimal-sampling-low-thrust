using LinearAlgebra
using SatelliteDynamics

LEO = 1500000

oe = [LEO, 0.2, 0.16, 0.28, 0.9, 0.8]

s = sOSCtoCART(oe)


function generateSPDmatrix(n)

    
    A = rand(n,n)
    

    A = 0.5*(A+A')
    A = A*A'

    A = A + n*I

    return A
    
end

function clohessy_wiltshire_matrices()

    # Clohessy, W. H.; Wiltshire, R. S. (1960). "Terminal Guidance System for Satellite Rendezvous"
    # Assumptions: circular orbit and x,y,z (relative distance) << r0 (distance to central body)
    sma = 2000000
    m=100
    n = sqrt(3.986004418e14 / sma^3)
    A = [0 0 0 1 0 0; 
        0 0 0 0 1 0;
        0 0 0 0 0 1;
        3*n^2 0 0 0 2*n 0;
        0 0 0 -2*n 0 0;
        0 0 -n^2 0 0 0]
    
    B = [zeros(3, 3); Matrix(I, 3, 3)]./m
    return A, B

end


function discretized_clohessy_wiltshire(dt::Real)
    A, B = clohessy_wiltshire_matrices()

    nx = size(A)[1]
    nu = size(B)[2]

    M = zeros(nx+nu,nx+nu)
    M[1:nx,1:nx] = A
    M[1:nx,1+nx:nx+nu] = B

    M = exp(M*dt)

    Ad = M[1:nx,1:nx]
    Bd = M[1:nx,1+nx:nx+nu]

    return Ad, Bd

end


using ControlSystems
Q = diagm(ones(6))
R = diagm(ones(3))

dt = 1
Ad,Bd = discretized_clohessy_wiltshire(dt)

K = dlqr(Ad, Bd, Q, R)


N = 150
x = zeros(N, 6)
#x[1,:] = [20.0, 11, 16, -2, -1, 0.8]

xi = [-2.17495, 2.00354, 0.211689, 0.20699, -0.134153, -0.0807519]
#xi = [-2.17495, 2.00354, 0.211689, 0,0,0]
x[1,:] = xi
xg = [-3, 2, 1.3, -0.2, 0.3, -0.2]
xg = [-3, 2, 1.3, -0.2, 0, 0]

for i=1:N-1
    u = - K * (x[i, :] - xg)
    x[i+1,:] = Ad * x[i, :] + Bd * u
end
