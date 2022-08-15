% --- PHYSICAL OBJECTS ---

NewtonianFrame N
RigidFrame A % yawed with the left
RigidFrame R % yawed and rolled with scooter
RigidBody B % LEFT body
RigidBody C % steering column
RigidBody D % front wheel
RigidBody E % back wheel

Point DN ( D ) % contact point between front wheel and ground
Point EN ( E ) % contact point between back wheel and ground

% --- MATHEMATICAL DECLARATIONS ---
Variable qA'' % yaw angle
Variable qR'' % roll angle
Variable qB'' % pitch angle
% Variable qC'' % steering angle
Variable vx' % Back wheel's center's velocity in N in Bx>
Variable vy' % Back wheel's center's velocity in N in By>
Variable vz' % Back wheel's center's velocity in N in Bz>
Variable wD' % front wheel angular velocity
Variable wE' % back wheel angular velocity

Specified TC % torque on steering column from body
Specified TE % torque on back wheel from body

Constant g = 9.81 m/s^2

% Chassis dimensions
Constant L = 0.6 m % distance from back wheel axle to steering column
Constant TRAIL = 0.1 m % distance from steering column to front wheel center

Constant Bcm_x = 0.3 m % Position of Bcm in B in Bx
Constant Bcm_z = 0.3 m % Position of Bcm in B in Bz

Constant Lcm_z = 0.4 m % Position of Ccm in C in Cz

% Mass properties
Constant mB = 20 kg % Body mass TODO
Constant IBxx = 1 kg*m^2 % TODO
Constant IByy = 1 kg*m^2 % TODO
Constant IBzz = 1 kg*m^2 % TODO

Constant mC = 0 kg % steering column mass
Constant ICxx_in_B = 0 kg*m^2 % TODO
Constant ICyy_in_B = 0 kg*m^2 % TODO
Constant ICzz_in_B = 0 kg*m^2 % TODO

Constant mD = 0 kg % front wheel mass TODO
Constant R_hub_D = 8 cm % front wheel hub radius
Constant R_tire_D = 2 cm % front wheel tire radius
Constant IDxx_in_C = mD / 4 * R_hub_D^2 
Constant IDyy_in_C = mD / 2 * R_hub_D^2 
Constant IDzz_in_C = mD / 4 * R_hub_D^2 

Constant mE = 0 kg % back wheel mass TODO
Constant R_hub_E = 8 cm % front wheel hub radius
Constant R_tire_E = 2 cm % front wheel tire radius
Constant IExx_in_B = mE / 4 * R_hub_E^2 
Constant IEyy_in_B = mE / 2 * R_hub_E^2 
Constant IEzz_in_B = mE / 4 * R_hub_E^2 

% --- ASSIGN MASS AND INERTIA PROPERTIES ---
B.SetMassInertia(mB, IBxx, IByy, IBzz)
C.SetMassInertia(0, 0, 0, 0)
D.SetMassInertia(0, 0, 0, 0)
E.SetMassInertia(0, 0, 0, 0)
% C.SetMassInertia(mC, Ccm, B, ICxx_in_B, ICyy_in_B, ICzz_in_B)
% D.SetMassInertia(mD, Dcm, C, IDxx_in_C, IDyy_in_C, IDzz_in_C)
% E.SetMassInertia(mE, Ecm, B, IExx_in_B, IEyy_in_B, IEzz_in_B)

% --- ROTATIONAL AND TRANSLATIONAL KINEMATICS ---
% Yawed rigid frame
A.RotateZ( N, qA )

% % Pitched rigid frame
R.RotateX( A, qR )

% Chassis B
B.RotateY( R, qB )
Bo.SetVelocityAcceleration(N, vx * Bx> + vy * By> + vz * Bz>)
Bcm.Translate( Bo, Bcm_x * Bx> + Bcm_z * Bz> )

% Steering column
% C.RotateZ (B, qC )
C.RotateZ (B, 0)
Co.Translate( Bo, L * Bx> )
Ccm.Translate( Co, Lcm_z * Bz> )

% Front wheel
Do.Translate(Co, -TRAIL * Cx> )
Dcm.Translate(Do, 0>)
D.SetAngularVelocity( C, wD * Cy> ) % Could be in N or 

% Back wheel
Eo.Translate(Bo, 0> )
Ecm.Translate(Eo, 0> )
E.SetAngularVelocity( B, wE * By> )

% Contact points
% DN.SetPositionVelocity(Do, -R_hub_D * Cz> - R_tire_D * Nz> ) % front wheel
% EN.SetPositionVelocity(Eo, -R_hub_E * Bz> - R_tire_E * Nz> ) % back wheel
DN.SetPositionVelocity(Do, -R_hub_D * Cz> ) % front wheel
EN.SetPositionVelocity(Eo, -R_hub_E * Bz> ) % back wheel

% --- MOTION CONSTRAINTS ---

% Back wheel rolling constraints in Bxyz components
RollingConstraint[1] = Dot( EN.GetVelocity(N), Bx> )
RollingConstraint[2] = Dot( EN.GetVelocity(N), By> )
RollingConstraint[3] = Dot( EN.GetVelocity(N), Bz> )

% Front wheel rolling constraints in Cxyz components
RollingConstraint[4] = Dot( DN.GetVelocity(N), Cx> )
RollingConstraint[5] = Dot( DN.GetVelocity(N), Cy> )
RollingConstraint[6] = Dot( DN.GetVelocity(N), Cz> )

% --- EMBED CONSTRAINTS ---
SolveDt( RollingConstraint = 0, qA', qB', vy, vz, wD, wE )

% SolveDt( RollingConstraint = 0, vz, vy, wE)

% ADD LOOP CONSTRAINT

% --- ADD RELEVANT FORCES AND TORQUES ---
System.AddForceGravity( -g * Nz> )
C.AddTorque(B, TC * Bz> )
E.AddTorque(B, TE * By> )

% --- FORM KANE'S EQUATIONS OF MOTION ---
% SetGeneralizedSpeed(vx, qB', qC')
SetGeneralizedSpeed(vx, qR')
% SetGeneralizedSpeed(vx, qB', qC', wD, qA', qP')
KaneDynamics = System.GetKaneDynamics()
Evaluate(KaneDynamics, qB = 0 deg)
Evaluate(KaneDynamics, R_hub_E = R_hub_D, qB = 0 deg)
% Solve( KaneDynamics = 0, qA'', qB'', vx' )
