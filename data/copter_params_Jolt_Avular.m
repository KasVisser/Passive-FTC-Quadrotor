%  ** Parameter file of quadcopter (Avular Jolt) **
%
% Based on copter_params_Minnie_AIAA_JGCD_2022.m. Source of each value:
%   [Avular]  value provided by Avular (same as relaxed_hover_sweep.m /
%             landing_options.m in 4. MATLAB/Allocation_Controller_Sim)
%   [est]     ESTIMATE -- replace once Avular provides the real value
%
% Usage: in scripts/init_Minnie_Loiter_FTC.m replace the copterLoadParams line by
%   copter = copterLoadParams( 'copter_params_Jolt_Avular' );
% and set IC.omega_mot to the hover speed (approx. 980 rad/s, see below).
%
% Rotor numbering and spin directions follow the Minnie file (rotor 1 front
% at -y, rotor 2 front at +y, ...), so failure_time_mot_N keeps its meaning
% in this simulation. Check this against the numbering used in the thesis.
% *************************************************************************

%% body parameters
% mass of vehicle (flight ready with batteries, no payload), in kg   [Avular]
copter.body.m = 4.022;
% inertial matrix of vehicle, in kg.m^2                                [Avular]
% NOTE: Izz > Ixx + Iyy violates the triangle inequality of a rigid body,
% so at least one of these values is probably off -- confirm with Avular.
I_13 = 0;
I_12 = 0;
I_23 = 0;
copter.body.I = [0.107, I_12, I_13; ...
            I_12, 0.139, I_23; ...
            I_13, I_23, 0.37];

%% configuration parameters
% center of gravity position in c frame, in m                         [est]
copter.config.CoG_Pos_c = [ 0; 0; 0 ];
% position of all propellers in c frame, in m
bx = 0.178;     % [Avular]
by = 0.120;     % [Avular]
bz = 0;         % [est] rotor plane at CoG height
copter.config.propPos_c = [ bx, -by, bz; ...
                            bx, by, bz; ...
                            -bx, by, bz; ...
                            -bx, -by, bz ]';
% direction of all propeller rotations, -1: right, 1 left            [est]
copter.config.propDir = [ -1; 1; -1; 1 ];
% orientation of the motors relative to the body
copter.config.M_b_prop1 = euler2Dcm([0,-pi/2,0]);
copter.config.M_b_prop2 = euler2Dcm([0,-pi/2,0]);
copter.config.M_b_prop3 = euler2Dcm([0,-pi/2,0]);
copter.config.M_b_prop4 = euler2Dcm([0,-pi/2,0]);
% hit points for ground model (landing gear ~0.10 m below rotors)     [est]
copter.config.hitPoints_c = [ copter.config.propPos_c + [ 0; 0; 0.10 ], ...
                           copter.config.propPos_c + [ 0; 0; -0.03 ] ];

%% propeller parameters
% propeller name specifying propeller map (name must be inside database) [est]
% The adjacent motors are 2*by = 0.24 m apart, so props larger than ~9.4"
% would overlap; 9x4.5MR is the largest multirotor prop in the database
% that fits. It gives c = d/k = 0.0156 m (torque-to-thrust ratio), well
% below the c = 0.05 m used as estimate in the thesis.
copter.prop.name = '9x4.5MR';
% factor to adjust thrust and torque (e.g. due to mounting)           [est]
copter.prop.correction_factor = 1;
% propeller + motor bell moment of inertia, kg.m^2                    [est]
% (~15 g 9" prop: 1/12*m*D^2 = 6.5e-5, plus ~1e-5 for the motor bell)
copter.prop.I = 1/12*0.015*0.2286^2 + 1e-5;

%% motor parameters
% torque constant of the motor (KT=60/(2*pi*KV)), N.m/A              [est]
% with KV = 820 RPM/V, chosen so that the static max thrust on 6S is
% approx. 30 N per rotor (the T_max estimate of the thesis);
% hover approx. 9.9 N per rotor at ~9350 rpm
copter.motor.KT = 60/(2*pi*820);
% motor internal resistance, Ohm                                      [est]
copter.motor.R = 0.06;

%% battery parameters
% battery voltage (6S nominal), V                                     [est]
copter.bat.V = 22.2;

%% aerodynamics                                                       [est]
% (scaled from the IRIS parameters in ladac-examples-data)
% reference surface of multicopter, in m^2
copter.aero.S = 0.06;
% minimum drag coefficient, in 1
copter.aero.C_Dmin = 0.5;
% maximum drag coefficient, in 1
copter.aero.C_Dmax = 1.0;
% maximum lift coefficient (whole vehicle), in 1
copter.aero.C_Lmax = 0.05;
% center of pressure (x_b, y_b direction), in m
copter.aero.CoP_xy = 0.05;
% center of pressure (z_b direction), in m
copter.aero.CoP_z = 0;
% damping moment coefficient for rotation, in N/(m^2*rad/s)
% Chosen so that rate_damp*S^1.5 = 0.2 N.m.s/rad, the rotational drag
% gamma of Avular (yaw time constant Izz/gamma = 1.85 s). This assumes the
% damping moment in multicopter_aerodynamics_simple_lib scales with S^1.5
% at sea-level density -- verify in that block.
copter.aero.rate_damp = 0.2 / copter.aero.S^1.5;
