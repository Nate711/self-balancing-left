NewtonianFrame N
RigidBody A
Point Ap (A)
Constant L = 1 m
Constant F = 1 N*m
Variable qA''
SetGeneralizedSpeed(qA')
% Constant wAx, 
% A.SetAngularVelocity(N, wAx * Ax> + wAy * Ay> + wAz * Az> )
A.RotateX(N, qA)
Ao.Translate(No, 0>)
Ap.Translate(Ao, L * Az> )
Ap.AddForce(F * Ay> )

System.GetGeneralizedForce()