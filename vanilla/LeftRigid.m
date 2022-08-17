% --- LEFT RIGID ---
% SetWorkingDirectory(~/Documents/ME 331)
% run LeftRigid.m

% --- FRAMES, BODIES, POINTS ---
NewtonianFrame N
RigidFrame A, B
RigidBody C, D
Point CN(C)
Point P()

% --- DECLARED QUANTITIES ---
Constant mWheel = 2 kg
Constant mD = 20 kg

Constant g = 9.8 m/s^2 

Constant R  = 0.1 m
Constant LD = 0.76 m
Constant HD = 0.61 m
Constant WD = 0.14 m

Variable qA'', qB'', wC', qD''
Variable v'

% --- SET INERTIA PROPERTIES ---
C.SetMass(mWheel)
C.SetInertia(Ccm, B, J = 0.25*mWheel*R^2, I = 2*J, J)
D.SetMass(mD)
D.SetInertia(Dcm, 1/12*mD*(HD^2+WD^2), 1/12*mD*(LD^2+HD^2), 1/12*mD*(LD^2+WD^2));

% --- ROTATIONAL KINEMATICS ---
A.RotatePositiveZ(N,qA)
B.RotatePositiveX(A,qB)
C.SetAngularVelocityAcceleration(B,wC*By>)
D.RotateNegativeY(B,qD)

% --- TRANSLATIONAL KINEMATICS ---
P.SetVelocityAcceleration(N,v*Ax>)
Ccm.Translate(P,R*Bz>)
CN.SetPositionVelocity(Ccm ,-R*Bz>)
Do.Translate(Ccm,0>)
Dcm.Translate(Do,LD/2*Dx> + HD/2*Dz>)

% --- ROLLING CONSTRAINTS ---
RollingConstraint[1] = Dot(CN.GetVelocity(N), Ax>)
SolveDt(RollingConstraint = 0,  v)

% --- ADD RELEVANT FORCES ---
Ccm.AddForce(-mWheel*g*Nz>)
Dcm.AddForce(-mD*g*Nz>)

% --- KANE DYNAMICS ---
SetGeneralizedSpeed(qA', qB', wC, qD')
KaneDynamics = System.GetDynamicsKane()
% Solve(KaneDynamics = 0, qA'', qB'', wC', qD'')

% --- CHECK SYSTEM ENERGY ---
sysPower = System.GetPower()
KE = System.GetKineticEnergy()
Variable work' = sysPower
Input work = 0
ME = KE - work

% --- INPUT ---
Input qA = 0 deg
Input qA' = 0 deg/sec
Input qB = 10 deg
Input qB' = 0 deg/sec
Input wC = 10 rad/sec
Input qD = 90 deg
Input qD' = 0 deg/sec

% --- POST PROCESS ---
Variable qC' = wC
Variable xC' = Dot(Nx>,v_Ccm_N>)
Variable yC' = Dot(Ny>,v_Ccm_N>)
Variable zC' = Dot(Nz>,v_Ccm_N>)
Variable xD' = Dot(Nx>,v_Dcm_N>)
Variable yD' = Dot(Ny>,v_Dcm_N>)
Variable zD' = Dot(Nz>,v_Dcm_N>)

Input qC = 0 rad

Input xC = EvaluateToNumber(Dot(Nx>, R*Bz>))
Input yC = EvaluateToNumber(Dot(Ny>, R*Bz>))
Input zC = EvaluateToNumber(Dot(Nz>, R*Bz>)) 
Input xD = EvaluateToNumber(Dot(Nx>, R*Bz> + LD/2*Dx> + HD/2*Dz>))
Input yD = EvaluateToNumber(Dot(Ny>, R*Bz> + LD/2*Dx> + HD/2*Dz>))
Input zD = EvaluateToNumber(Dot(Nz>, R*Bz> + LD/2*Dx> + HD/2*Dz>))

% --- INTEGRATION PARAMETERS ---
Input  tFinal = 2 sec,  tStep = 0.005 sec,  absError = 1.0E-08
Output t sec, xC m, yC m, zC m, xD m, yD m, zD m, qA deg, qB deg, qC deg, qD deg, ME J

% --- SOLVE ---
SystemEqns = [KaneDynamics]
ODE(SystemEqns := 0, qA'', qB'', wC', qD'') MGLeftRigid