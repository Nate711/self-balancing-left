% MotionGenesis file:  RollingDiskDynamics.txt
% Copyright (c) 2018 Motion Genesis LLC.  All rights reserved.
% Problem: Simulation of a rolling disk.
% SetAutoZee( ON )

%--------------------------------------------------------------------
NewtonianFrame  A                 % Horizontal plane.
RigidFrame      B, C              % Heading and tilt reference frames.
RigidBody       D                 % Back wheel disk.
RigidBody       E                 % Chassis
Point           DA( D )           % Point of D in contact with A.

RigidFrame      F, G              % heading and tilt frames for front wheel
RigidBody       H                 % Front wheel
RigidBody       I                 % Steering column
Point           HA( H )

%--------------------------------------------------------------------
Variable  wDx',  wDy',  wDz'      % Disk D's angular velocity in A.
Variable  x'',  y''               % Locates contact point DA from Ao.
Variable  qH',  qL',  qS'         % Heading angle, lean angle, spin angle.
Variable  qE''                    % Pitch angle of chassis relative to carrier frame C

Variable  wHx',  wHy',  wHz'      % Disk H's angular velocity in A.
Variable  xH'',  yH''             % Locates contact point HA from Ao.
Variable  qHH',  qHL',  qHS'      % Heading angle, lean angle, spin angle of fron twheel
Variable  qI''                    % Pitch angle of steering column relative to carrier frame G


Constant  r = 0.1 meters          % Radius of disk 
Constant  g = 9.8   m/s^2         % Earth's gravitational acceleration.
Constant  b = 0 N*m*s/rad         % Aerodynamic damping on disk.
Constant  LI = 1 m                % length of steering column
Constant  RI = 0.01 m             % Diameter of steering column
Constant  TRAIL = - 0.1 m         % Steering column trail

Constant mD = 1 kg
Constant IDxx = mD / 4 * r^2
Constant IDyy = mD / 2 * r^2
Constant IDzz = mD / 4 * r^2

Constant m_E = 20 kg
Constant LE = 0.76 m
Constant HE = 0.61 m
Constant WE = 0.14 m 
Constant IExx = 1/12 * m_E * (HE^2 + WE^2)
Constant IEyy = 1/12 * m_E * (LE^2 + HE^2)
Constant IEzz = 1/12 * m_E * (LE^2 + WE^2)

Constant mH = 1 kg
Constant IHxx = mH / 4 * r^2
Constant IHyy = mH / 2 * r^2
Constant IHzz = mH / 4 * r^2

Constant mI = 2 kg
Constant IIxx = mI / 12 * LI^2
Constant IIyy = mI / 12 * LI^2
Constant IIzz = mI / 2 * RI^2

D.SetMassInertia( mD, Dcm, C, IDxx, IDyy, IDzz )
E.SetMassInertia( m_E, IExx, IEyy, IEzz )

H.SetMassInertia( mH, Hcm, G, IHxx, IHyy, IHzz )
I.SetMassInertia( mI, IIxx, IIyy, IIzz )

%--------------------------------------------------------------------
%   Rotational kinematics.
B.RotatePositiveZ( A, qH )
C.RotateNegativeX( B, qL )
D.RotatePositiveY( C, qS )
E.RotatePositiveY( C, qE )

F.RotatePositiveZ( A, qHH )
G.RotateNegativeX( F, qHl )
H.RotatePositiveY( G, qHS )
I.RotatePositiveY( G, qI )

%---------------------------------------------------------------
%   For efficient dynamics, make a change of variable.
wDA> = D.GetAngularVelocity( A )
ChangeVariables[1] = wDx - Dot( wDA>, Cx> )
ChangeVariables[2] = wDy - Dot( wDA>, Cy> )
ChangeVariables[3] = wDz - Dot( wDA>, Cz> )
Solve( ChangeVariables = 0,  qH', qL', qS' )

wHA> = H.GetAngularVelocity ( A )
ChangeVariablesH[1] = wHx - Dot( wHA>, Gx> )
ChangeVariablesH[2] = wHy - Dot( wHA>, Gy> )
ChangeVariablesH[3] = wHz - Dot( wHA>, Gz> )
Solve( ChangeVariablesH = 0,  qHH', qHL', qHS' )

%---------------------------------------------------------------
%   Use the simpler version of angular velocity from here on.
D.SetAngularVelocityAcceleration( A, wDx * Cx> + wDy * Cy> + wDz * Cz> )
H.SetAngularVelocityAcceleration( A, wHx * Gx> + wHy * Gy> + wHz * Gz> )

% Can we do something more efficient for the chassis?

%--------------------------------------------------------------------
%   Form angular acceleration (differentiating in D is more efficient).
% alf_D_A> = Dt( D.GetAngularVelocity(A), D )
% alf_H_A> = Dt( H.GetAngularVelocity(A), H )

%---------------------------------------------------------------
%   Translational kinematics.
Dcm.Translate( Ao,  x*Ax> + y*Ay> + r*Cz> )
DA.SetPositionVelocity( Dcm,  -r*Cz> )

Eo.Translate( Dcm, 0> )
Ecm.Translate( Eo, LE/2 * Ex> + HE/2 * Ez> )

Hcm.Translate( Ao, xH * Ax> + yH * Ay> + r * Gz> )
HA.SetPositionVelocity( Hcm, -r * Gz> )

Io.Translate( Hcm, 0> )
Icm.Translate( Io, -TRAIL * Ix> + LI/2 * Iz> )

%--------------------------------------------------------------------
%   v_DA_A> is simpler if explicit in wDx, wDy, wDz.
Explicit( v_DA_A>,   wDx, wDy, wDz )
Explicit( v_HA_A>,   wHx, wHy, wHz )

%---------------------------------------------------------------
%   Motion constraints (D rolls on A at point DA).
RollingConstraint[1] = Dot( DA.GetVelocity(A), Bx> )
RollingConstraint[2] = Dot( DA.GetVelocity(A), By> )
SolveDt( RollingConstraint = 0,  x', y' )

RollingConstraintH[1] = Dot( HA.GetVelocity(A), Fx> )
RollingConstraintH[2] = Dot( HA.GetVelocity(A), Fy> )
SolveDt( RollingConstraintH = 0,  xH', yH' )

%--------------------------------------------------------------------
%   Motion constraints make v_Dcm_A> and a_Dcm_A> simpler if expressed in C.
v_Dcm_A> := Express( Explicit(v_Dcm_A>, wDx, wDy, wDz),  C )
a_Dcm_A> := Express( Explicit(a_Dcm_A>, wDx, wDy, wDz),  C )

v_Ecm_A> := Express( Explicit(v_Ecm_A>, wDx, wDy, wDz), C )
a_Ecm_A> := Express( Explicit(a_Ecm_A>, wDx, wDy, wDz), C )

v_Hcm_A> := Express( Explicit(v_Hcm_A>, wHx, wHy, wHz),  G )
a_Hcm_A> := Express( Explicit(a_Hcm_A>, wHx, wHy, wHz),  G )

v_Icm_A> := Express( Explicit(v_Icm_A>, wHx, wHy, wHz), G )
a_Icm_A> := Express( Explicit(a_Icm_A>, wHx, wHy, wHz), G )

%---------------------------------------------------------------
%   Add relevant contact and distance force/torques.
%   Note: Aerodynamic damping affects wDx most, wDy least.
System.AddForceGravity( -g * Az> )

%------------------------------------------------------------
%   Optional: Form and simplify Kane's equations of motion.
SetGeneralizedSpeed( wDx, wDy, wDz, qE', wHx, wHy, wHz, qI')
KaneDynamics = System.GetDynamicsKane()
% FactorQuadratic( KaneDynamics,  wDx, wDy, wDz, qE')

sysPower = System.GetPower()
Variable KE = System.GetKineticEnergy()
Variable work' = sysPower
Input work = 0
ME = KE - work

%--------------------------------------------------------------------
%   Integration parameters and initial values.
% --- INPUT ---
Input qH = 0 deg
Input wDz = 0 deg/sec
Input qL = 10 deg
Input wDx = 0 deg/sec
Input qS = 0 deg
Input wDy = 10 rad/sec
Input qE = 0 deg
Input qE' = 0 deg/sec

Input qHH = 0 deg
Input wHz = 0 deg/sec
Input qHL = 10 deg
Input wHx = 0 deg/sec
Input qHS = 0 deg
Input wHy = 10 rad/sec
Input qI = 0 deg
Input qI' = 0 deg/sec

Input x = 0 m
Input y = 0 m

Input xH = 1 m
Input yH = 0 m

% --- POST PROCESS ---
Variable xD = Dot( Dcm.GetPosition(Ao), Ax> )
Variable yD = Dot( Dcm.GetPosition(Ao), Ay> )
Variable zD = Dot( Dcm.GetPosition(Ao), Az> )

Variable xE = Dot( Ecm.GetPosition(Ao), Ax> )
Variable yE = Dot( Ecm.GetPosition(Ao), Ay> )
Variable zE = Dot( Ecm.GetPosition(Ao), Az> )

Variable xHcm = Dot( Hcm.GetPosition(Ao), Ax> )
Variable yHcm = Dot( Hcm.GetPosition(Ao), Ay> )
Variable zHcm = Dot( Hcm.GetPosition(Ao), Az> )

Variable xI = Dot( Icm.GetPosition(Ao), Ax> )
Variable yI = Dot( Icm.GetPosition(Ao), Ay> )
Variable zI = Dot( Icm.GetPosition(Ao), Az> )

%---------------------------------------------------------------
%   List output quantities and solve ODEs.
Input  tFinal = 2 sec,  tStep = 0.01 sec,  absError = 1.0E-08
Output t sec, xD m, yD m, zD m, xE m, yE m, zE m, xHcm m, yHcm m, zHcm m, xI m, yI m, zI m, qH deg, qL deg, qS deg, qE deg, qHH deg, qHL deg, qHS deg, qI deg, ME J
ODE(KaneDynamics=0, wDx', wDy', wDz', qE'', wHx', wHy', wHz', qI'') MGLeftRigidDouble_MG
%--------------------------------------------------------------------
Quit

