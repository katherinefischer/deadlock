%% Deadlock Adaptive Cruise Controller (Simplified Constant-Speed Lead)
% Clean rebuild from scratch

clc; clear; close all;

%% --- Delete any existing model first ---
if bdIsLoaded('Deadlock_ACC')
    close_system('Deadlock_ACC', 0);
end
if exist('Deadlock_ACC.slx', 'file')
    delete('Deadlock_ACC.slx');
end

%% --- Create new model ---
model = 'Deadlock_ACC';
new_system(model);
open_system(model);

%% --- Controller parameters ---
s0 = 5;         % Standstill distance (m)
Tgap = 1.4;     % Desired time gap (s)
k1 = 0.3;       % Gap error gain
k2 = 0.8;       % Relative velocity gain
amin = -3;      % Min acceleration (m/s^2)
amax = 2;       % Max acceleration (m/s^2)

%% ==============================================================
% 1. Lead Vehicle Subsystem (Constant 20 m/s, starts 40 m ahead)
% ==============================================================
lead = [model '/Lead_Vehicle'];
add_block('simulink/Ports & Subsystems/Subsystem', lead, 'Position', [100 100 250 250]);
open_system(lead);

% Constant velocity
add_block('simulink/Sources/Constant', [lead '/Lead_Velocity'], ...
    'Value', '20', 'Position', [100 100 130 130]);

% Integrate to get position (start 40 m ahead)
add_block('simulink/Continuous/Integrator', [lead '/Integrator_Pos'], ...
    'Position', [200 100 240 130]);
set_param([lead '/Integrator_Pos'], 'InitialCondition', '40');

% Outputs
add_block('simulink/Sinks/Out1', [lead '/lead_vel'], 'Position', [300 80 330 100]);
add_block('simulink/Sinks/Out1', [lead '/lead_pos'], 'Position', [300 120 330 140]);

% Connections
add_line(lead, 'Lead_Velocity/1', 'Integrator_Pos/1');
add_line(lead, 'Lead_Velocity/1', 'lead_vel/1');
add_line(lead, 'Integrator_Pos/1', 'lead_pos/1');

%% ==============================================================
% 2. Ego Vehicle Subsystem
% ==============================================================
ego = [model '/Ego_Vehicle'];
add_block('simulink/Ports & Subsystems/Subsystem', ego, 'Position', [400 100 650 400]);
open_system(ego);

% Inputs & outputs
add_block('simulink/Sources/In1', [ego '/lead_pos'], 'Position', [50 80 90 100]);
add_block('simulink/Sources/In1', [ego '/lead_vel'], 'Position', [50 120 90 140]);
add_block('simulink/Sinks/Out1', [ego '/ego_pos'], 'Position', [600 80 640 100]);
add_block('simulink/Sinks/Out1', [ego '/ego_vel'], 'Position', [600 120 640 140]);
add_block('simulink/Sinks/Out1', [ego '/a_cmd_guarded'], 'Position', [600 160 640 180]);
add_block('simulink/Sinks/Out1', [ego '/gap_ref'], 'Position', [600 200 640 220]);

% Dynamics
add_block('simulink/Continuous/Integrator', [ego '/Int_Vel'], 'Position', [450 120 490 150]);
add_block('simulink/Continuous/Integrator', [ego '/Int_Pos'], 'Position', [500 80 540 110]);

% Safety filter
add_block('simulink/Discontinuities/Saturation', [ego '/SafetyFilter'], 'Position', [280 250 320 280]);
set_param([ego '/SafetyFilter'], 'UpperLimit', num2str(amax), 'LowerLimit', num2str(amin));

% Controller logic
add_block('simulink/Math Operations/Gain', [ego '/k1_gain'], 'Position', [200 200 240 230]);
set_param([ego '/k1_gain'], 'Gain', num2str(k1));
add_block('simulink/Math Operations/Gain', [ego '/k2_gain'], 'Position', [200 230 240 260]);
set_param([ego '/k2_gain'], 'Gain', num2str(k2));
add_block('simulink/Math Operations/Sum', [ego '/Sum_accel'], 'Inputs', '++', 'Position', [250 210 280 250]);

% Error calc
add_block('simulink/Math Operations/Sum', [ego '/Sum_error'], 'Inputs', '+-', 'Position', [120 160 150 190]);

% Reference generator
add_block('simulink/Math Operations/Gain', [ego '/Gain_T'], 'Position', [120 120 150 150]);
set_param([ego '/Gain_T'], 'Gain', num2str(Tgap));
add_block('simulink/Math Operations/Sum', [ego '/Sum_ref'], 'Inputs', '++', 'Position', [170 120 200 150]);
add_block('simulink/Sources/Constant', [ego '/Const_s0'], 'Value', num2str(s0), 'Position', [120 80 150 100]);

% Gap and relative velocity
add_block('simulink/Math Operations/Subtract', [ego '/GapCalc'], 'Position', [100 80 130 100]);
add_block('simulink/Math Operations/Subtract', [ego '/RelVelCalc'], 'Position', [100 200 130 220]);

% Internal connections
add_line(ego, 'lead_pos/1', 'GapCalc/1');
add_line(ego, 'lead_vel/1', 'RelVelCalc/1');
add_line(ego, 'Int_Pos/1', 'GapCalc/2');
add_line(ego, 'Int_Vel/1', 'RelVelCalc/2');
add_line(ego, 'RelVelCalc/1', 'Gain_T/1');
add_line(ego, 'Gain_T/1', 'Sum_ref/2');
add_line(ego, 'Const_s0/1', 'Sum_ref/1');
add_line(ego, 'Sum_ref/1', 'gap_ref/1');
add_line(ego, 'Sum_ref/1', 'Sum_error/2');
add_line(ego, 'GapCalc/1', 'Sum_error/1');
add_line(ego, 'Sum_error/1', 'k1_gain/1');
add_line(ego, 'RelVelCalc/1', 'k2_gain/1');
add_line(ego, 'k1_gain/1', 'Sum_accel/1');
add_line(ego, 'k2_gain/1', 'Sum_accel/2');
add_line(ego, 'Sum_accel/1', 'SafetyFilter/1');
add_line(ego, 'SafetyFilter/1', 'Int_Vel/1');
add_line(ego, 'Int_Vel/1', 'Int_Pos/1');
add_line(ego, 'Int_Pos/1', 'ego_pos/1');
add_line(ego, 'Int_Vel/1', 'ego_vel/1');
add_line(ego, 'SafetyFilter/1', 'a_cmd_guarded/1');

%% ==============================================================
% 3. Scopes & Logging
% ==============================================================
add_block('simulink/Sinks/Scope', [model '/Scope_Accel'], ...
    'Position', [750 100 800 140], 'NumInputPorts', '1');
add_block('simulink/Sinks/Scope', [model '/Scope_Velocities'], ...
    'Position', [750 160 800 200], 'NumInputPorts', '2');
add_block('simulink/Sinks/Scope', [model '/Scope_Gaps'], ...
    'Position', [750 220 800 260], 'NumInputPorts', '2');
add_block('simulink/Sinks/To Workspace', [model '/LogData'], ...
    'Position', [750 280 820 310]);
set_param([model '/LogData'], 'VariableName', 'simout', 'SaveFormat', 'StructureWithTime');

%% ==============================================================
% 4. Top-Level Wiring
% ==============================================================
add_line(model, 'Lead_Vehicle/1', 'Ego_Vehicle/1'); % lead_pos
add_line(model, 'Lead_Vehicle/2', 'Ego_Vehicle/2'); % lead_vel
add_line(model, 'Ego_Vehicle/3', 'Scope_Accel/1');  % acceleration

% Velocity comparison
add_line(model, 'Lead_Vehicle/2', 'Scope_Velocities/1');
add_line(model, 'Ego_Vehicle/2', 'Scope_Velocities/2');

% Gap comparison
add_block('simulink/Math Operations/Subtract', [model '/GapError'], 'Position', [680 240 720 270]);
add_line(model, 'Lead_Vehicle/1', 'GapError/1');
add_line(model, 'Ego_Vehicle/1', 'GapError/2');
add_line(model, 'GapError/1', 'Scope_Gaps/1');
add_line(model, 'Ego_Vehicle/4', 'Scope_Gaps/2');

% Logging
add_line(model, 'Ego_Vehicle/3', 'LogData/1');

%% Save and open
save_system(model);
disp('✅ Deadlock_ACC (Simplified Constant-Speed) model created successfully.');
open_system(model);
