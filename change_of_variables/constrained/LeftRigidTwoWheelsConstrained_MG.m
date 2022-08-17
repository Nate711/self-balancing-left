% MotionGenesis file:  RollingDiskDynamics.txt
% Copyright (c) 2018 Motion Genesis LLC.  All rights reserved.
% Problem: Simulation of a rolling disk.
%--------------------------------------------------------------------
NewtonianFrame  A                 % Horizontal plane.
RigidFrame      B, C              % Heading and tilt reference frames.
RigidBody       D                 % Rolling disk.
RigidBody       E                 % Chassis
RigidBody       G                 % Front wheel
Point           DA( D )           % Point of D in contact with A.
Point           GA( G )
%--------------------------------------------------------------------
Variable  wx',  wy',  wz'         % Disk D's angular velocity in A.
Variable  x'',  y''               % Locates contact point DA from No.
Variable  qH',  qL',  qS'         % Heading angle, lean angle, spin angle.
Variable  qE''                    % Pitch angle of chassis relative to carrier frame C
Variable  wG'                     % Angular speed of front wheel G in carrier frame C
Constant  r = 0.1 meters          % Radius of disk 
Constant  g = 9.8   m/s^2         % Earth's gravitational acceleration.

% Back wheel
Constant mD = 1 kg
Constant IDxx = mD / 4 * R^2
Constant IDyy = mD / 2 * R^2
Constant IDzz = mD / 4 * R^2

% Chassis
Constant m_E = 20 kg
Constant LE = 0.76 m
Constant HE = 0.61 m
Constant WE = 0.14 m 
Constant IExx = 1/12 * m_E * (HE^2 + WE^2)
Constant IEyy = 1/12 * m_E * (LE^2 + HE^2)
Constant IEzz = 1/12 * m_E * (LE^2 + WE^2)

% Front wheel
Constant LG = 0.76 m
Constant mG = 1 kg
Constant IGxx = mG / 4 * R^2
Constant IGyy = mG / 2 * R^2
Constant IGzz = mG / 4 * R^2

D.SetMassInertia( mD, Dcm, C, IDxx, IDyy, IDzz )
E.SetMassInertia( m_E, IExx, IEyy, IEzz )
G.SetMassInertia( mG, Gcm, C, IGxx, IGyy, IGzz )

%--------------------------------------------------------------------
%   Rotational kinematics.
B.RotatePositiveZ( A, qH )
C.RotateNegativeX( B, qL )
D.RotatePositiveY( C, qS )
E.RotatePositiveY( C, qE )
G.SetAngularVelocity( C, wG * Cy>)

%---------------------------------------------------------------
%   For efficient dynamics, make a change of variable.
wDA> = D.GetAngularVelocity( A )
ChangeVariables[1] = wx - Dot( wDA>, Cx> )
ChangeVariables[2] = wy - Dot( wDA>, Cy> )
ChangeVariables[3] = wz - Dot( wDA>, Cz> )
Solve( ChangeVariables = 0,  qH', qL', qS' )
%---------------------------------------------------------------
%   Use the simpler version of angular velocity from here on.
D.SetAngularVelocity( A,  wx*Cx> + wy*Cy> + wz*Cz> )
%--------------------------------------------------------------------
%   Form angular acceleration (differentiating in D is more efficient).
alf_D_A> = Dt( D.GetAngularVelocity(A), D )
%---------------------------------------------------------------
%   Translational kinematics.
Dcm.Translate( Ao,  x*Ax> + y*Ay> + r*Cz> )
DA.SetPositionVelocity( Dcm,  -r*Cz> )

Eo.Translate( Dcm, 0> )
Ecm.Translate( Eo, LE/2 * Ex> + HE/2 * Ez> )

Gcm.Translate( Eo, LG * Ex> )
GA.SetPositionVelocity( Gcm, -r * Cz> )

%--------------------------------------------------------------------
%   v_DA_A> is simpler if explicit in wx, wy, wz.
Explicit( v_DA_A>,   wx, wy, wz )
%---------------------------------------------------------------
%   Motion constraints (D rolls on A at point DA).
RollingConstraint[1] = Dot( DA.GetVelocity(A), Bx> )
RollingConstraint[2] = Dot( DA.GetVelocity(A), By> )
SolveDt( RollingConstraint = 0,  x', y' )

%--------------------------------------------------------------------
%   Motion constraints make v_Dcm_A> and a_Dcm_A> simpler if expressed in C.
v_Dcm_A> := Express( Explicit(v_Dcm_A>, wx, wy, wz),  C )
a_Dcm_A> := Express( Explicit(a_Dcm_A>, wx, wy, wz),  C )

v_Ecm_A> := Express( Explicit(v_Ecm_A>, wx, wy, wz), C )
a_Ecm_A> := Express( Explicit(a_Ecm_A>, wx, wy, wz), C )


%---------------------------------------------------------------
%   Front contact
%   Loop constraint
LoopConstraint = Dot( r * Cz> + LG * Ex> - r * Cz>, Az> )
SolveDt( Dt(LoopConstraint) = 0, qE')

% Rolling constraint
FrontRollingConstraint[1] = Dot( GA.GetVelocity(A), Bx> )
FrontRollingConstraint[2] = Dot( GA.GetVelocity(A), By> )
SolveDt( FrontRollingConstraint = 0, wz, wG)

%---------------------------------------------------------------
%   Add relevant contact and distance force/torques.
%   Note: Aerodynamic damping affects wx most, wy least.
System.AddForceGravity( -g * Az> )

%------------------------------------------------------------
%   Optional: Form and simplify Kane's equations of motion.
SetGeneralizedSpeed( wx, wy)
KaneDynamics = System.GetDynamicsKane()
% FactorQuadratic( KaneDynamics,  wx, wy, wz, qE')

sysPower = System.GetPower()
Variable KE = System.GetKineticEnergy()
Variable work' = sysPower
Input work = 0
ME = KE - work

%--------------------------------------------------------------------
%   Integration parameters and initial values.
% --- INPUT ---
Input qH = 0 deg
Input qL = 10 deg
Input wx = 0 deg/sec
Input qS = 0 deg
Input wy = 10 rad/sec
Input qE = 0 deg

Input x = 0 m
Input y = 0 m

% --- POST PROCESS ---
Variable xD = Dot( Dcm.GetPosition(Ao), Ax> )
Variable yD = Dot( Dcm.GetPosition(Ao), Ay> )
Variable zD = Dot( Dcm.GetPosition(Ao), Az> )

Variable xE = Dot( Ecm.GetPosition(Ao), Ax> )
Variable yE = Dot( Ecm.GetPosition(Ao), Ay> )
Variable zE = Dot( Ecm.GetPosition(Ao), Az> )

Variable xG = Dot( Gcm.GetPosition(Ao), Ax> )
Variable yG = Dot( Gcm.GetPosition(Ao), Ay> )
Variable zG = Dot( Gcm.GetPosition(Ao), Az> )

Variable qG' = wG
Input qG = 0 deg

%---------------------------------------------------------------
%   List output quantities and solve ODEs.
Input  tFinal = 2 sec,  tStep = 0.005 sec,  absError = 1.0E-08
Output t sec, xD m, yD m, zD m, xE m, yE m, zE m, xG m, yG m, zG m, qH deg, qL deg, qS deg, qE deg, qG deg, ME J
ODE(KaneDynamics=0, wx', wy') MGLeftRigidTwoWheelsConstrained_MG
%--------------------------------------------------------------------
Quit

