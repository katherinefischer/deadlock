function run_mvsim_tests()
    % ============================================================
    %  Multi-Vehicle Simulation Runner
    %  Runs 10 lead-vehicle scenarios on MVSimulink.slx
    %  Generates plots and a results table for each test.
    % ============================================================

    mdl = 'MVSimulink';   % your multi-vehicle Simulink model
    load_system(mdl);

    Ts = 0.1;       % sampling time (s)
    T  = 60;        % total simulation time (s)
    t  = (0:Ts:T)'; % nominal time vector

    % Define scenario names
    scenarioNames = [ ...
        "lead_stationary", "lead_constant_speed", "lead_brake_moderate", ...
        "lead_brake_hard", "lead_accel_after_stop", "lead_cutin_close", ...
        "lead_cutout", "stop_and_go", "vary_timegap", "noisy_lead"];

    % Initialize results storage
    ResultsCell = cell(length(scenarioNames),6);

    % ============================================================
    %  Main Simulation Loop
    % ============================================================
    for i = 1:length(scenarioNames)
        name = scenarioNames(i);
        fprintf('\n▶️  Running scenario %d: %s\n', i, name);

        % --- Generate lead vehicle trajectory ---
        [xlead_ts, vlead_ts, alead_ts] = lead_vehicle_profile(name, t);

        % --- Send signals to base workspace ---
        assignin('base','xlead_ts',xlead_ts);
        assignin('base','vlead_ts',vlead_ts);
        assignin('base','alead_ts',alead_ts);

        % --- Run simulation ---
        simOut = sim(mdl,'StopTime',num2str(T),'ReturnWorkspaceOutputs','on');

        % --- Extract To Workspace signals ---
        aData   = get_data(simOut.cmd_accel);
        vData   = get_data(simOut.ego_vel);
        gapData = get_data(simOut.sn);

        % --- Time vectors for each signal ---
        t_gap = get_time(simOut.sn);
        t_vel = get_time(simOut.ego_vel);
        t_acc = get_time(simOut.cmd_accel);

        % --- Crash / reversal detection ---
        if any(gapData < 0)
            warning("⚠️ Crash detected in scenario: %s (gap < 0)", name);
        end
        if any(vData < 0)
            warning("⚠️ Ego reversed in scenario: %s", name);
        end

        % --- Plot results ---
        figure('Name', char(name), 'NumberTitle', 'off');
        subplot(3,1,1);
        plot(t_gap, gapData, 'b', 'LineWidth', 1.3);
        ylabel('Gap (m)');
        title("Scenario: " + name); grid on;

        subplot(3,1,2);
        plot(t_vel, vData, 'r', 'LineWidth', 1.3);
        ylabel('Ego Vel (m/s)'); grid on;

        subplot(3,1,3);
        plot(t_acc, aData, 'k', 'LineWidth', 1.3);
        ylabel('Cmd Accel (m/s^2)');
        xlabel('Time (s)'); grid on;

        % --- Compute metrics ---
        hardBrakes = double(sum(aData < -3));
        negVel  = double(any(vData < 0));
        negGap  = double(any(gapData < 0));
        minGap  = double(min(gapData));
        maxAcc  = double(max(aData));

        ResultsCell(i,:) = {char(name), hardBrakes, negVel, negGap, minGap, maxAcc};

        fprintf('✅ Completed scenario: %s\n', name);
    end

    % ============================================================
    %  Output Summary
    % ============================================================
    Results = cell2table(ResultsCell, ...
        'VariableNames',{'Test','HardBrakes','NegVel','NegGap','MinGap','MaxAccel'});
    disp(Results);
    writetable(Results,'mvsim_results.csv');
    fprintf('\n✅ All multi-vehicle simulations completed. Results saved to mvsim_results.csv\n');
end

% =========================================================================
%  LEAD VEHICLE PROFILE GENERATOR
% =========================================================================
function [xlead_ts, vlead_ts, alead_ts] = lead_vehicle_profile(name, t)
    v0 = 20; 
    x0 = 0;
    vlead = zeros(size(t));

    switch name
        case "lead_stationary"
            vlead(:) = 0;

        case "lead_constant_speed"
            vlead(:) = v0;

        case "lead_brake_moderate"
            vlead(:) = v0;
            idx = (t >= 5 & t < 10);
            vlead(idx) = v0 - 2*(t(idx)-5);
            vlead(t >= 10) = 10;

        case "lead_brake_hard"
            vlead(:) = v0;
            idx = (t >= 5 & t < 7);
            vlead(idx) = v0 - 10*(t(idx)-5);
            vlead(t >= 7) = 0;

        case "lead_accel_after_stop"
            vlead(:) = 0;
            idx = (t >= 5 & t < 10);
            vlead(idx) = 4*(t(idx)-5);
            vlead(t >= 10) = v0;

        case "lead_cutin_close"
            vlead(:) = v0;
            vlead(t < 5) = NaN; 
            vlead(t >= 5) = 15;
            vlead = fillmissing(vlead,'previous');

        case "lead_cutout"
            vlead(:) = v0;
            vlead(t >= 8) = NaN; 
            vlead = fillmissing(vlead,'previous');

        case "stop_and_go"
            vlead(:) = v0;
            idx1 = (t >= 7 & t < 10);
            vlead(idx1) = v0 - (v0/3)*(t(idx1)-7);
            vlead(t >= 10 & t < 15) = 0;
            idx2 = (t >= 15 & t < 18);
            vlead(idx2) = (v0/3)*(t(idx2)-15);
            vlead(t >= 18) = v0;

        case "vary_timegap"
            vlead(:) = v0 + 2*sin(0.2*t);

        case "noisy_lead"
            vlead(:) = v0 + 0.5*randn(size(t));

        otherwise
            vlead(:) = v0;
    end

    % Clean up invalid values
        vlead(isnan(vlead) | isinf(vlead)) = 0;
        alead = [0; diff(vlead)./diff(t)];
        alead(isnan(alead) | isinf(alead)) = 0;
        
        % Compute position
        xlead = x0 + cumtrapz(t, vlead);
        
        % Create timeseries objects
        xlead_ts = timeseries(xlead, t);
        vlead_ts = timeseries(vlead, t);
        alead_ts = timeseries(alead, t);

end

% =========================================================================
%  HELPER: Extract Data
% =========================================================================
function data = get_data(sig)
    if istimetable(sig)
        data = sig.Variables;
    elseif isa(sig,'timeseries')
        data = sig.Data;
    elseif isstruct(sig) && isfield(sig,'signals')
        data = sig.signals.values;
    else
        data = double(sig);
    end
end

% =========================================================================
%  HELPER: Extract Time
% =========================================================================
function t = get_time(sig)
    if isa(sig, 'timeseries')
        t = sig.Time;
    elseif istimetable(sig)
        t = sig.Time;
    elseif isstruct(sig) && isfield(sig,'time')
        t = sig.time;
    else
        t = (0:numel(sig)-1)'; % fallback
    end
end
