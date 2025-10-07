function run_svsim_tests()
    mdl = 'SVSimulink';
    load_system(mdl);

    Ts = 0.1;
    T  = 50;
    t  = (0:Ts:T)';

    % Create empty cell matrix and convert to table at the end (bulletproof)
    ResultsCell = cell(10,6);

    for i = 1:10
        [xfront_ts, vlead_ts, name] = single_vehicle_scenario(i, t);

        % --- Ensure name is a proper string ---
        if iscell(name)
            name = name{1};
        end
        name = string(name);

        % --- Send signals to base workspace ---
        assignin('base','xfront_ts',xfront_ts);
        assignin('base','vlead_ts',vlead_ts);

        % --- Run the simulation ---
        simOut = sim(mdl,'StopTime',num2str(T),'ReturnWorkspaceOutputs','on');

        % --- Extract data from To Workspace blocks ---
        aData   = get_data(simOut.cmd_accel);
        vData   = get_data(simOut.ego_vel);
        gapData = get_data(simOut.sn);

        % --- Plot key scenarios only ---
        if ismember(i, [1,2,4,5,9])  % steady, lead_brake, stationary_obstacle, cutin_close, stop_and_go
            figure('Name', char(name), 'NumberTitle', 'off');

            gapTime = get_time(simOut.sn);
            vTime   = get_time(simOut.ego_vel);
            aTime   = get_time(simOut.cmd_accel);

            subplot(3,1,1);
            plot(gapTime, gapData, 'b', 'LineWidth', 1.3);
            ylabel('Gap (m)');
            title("Test: " + name);
            grid on;

            subplot(3,1,2);
            plot(vTime, vData, 'r', 'LineWidth', 1.3);
            ylabel('Ego Velocity (m/s)');
            grid on;

            subplot(3,1,3);
            plot(aTime, aData, 'k', 'LineWidth', 1.3);
            ylabel('Cmd Accel (m/s^2)');
            xlabel('Time (s)');
            grid on;
        end

        % --- Compute metrics ---
        hardBrakes = double(sum(aData < -3));
        negVel  = double(any(vData < 0));
        negGap  = double(any(gapData < 0));
        minGap  = double(min(gapData));
        maxAcc  = double(max(aData));

        % --- Store into cell array ---
        ResultsCell{i,1} = name;
        ResultsCell{i,2} = hardBrakes;
        ResultsCell{i,3} = negVel;
        ResultsCell{i,4} = negGap;
        ResultsCell{i,5} = minGap;
        ResultsCell{i,6} = maxAcc;

        % --- Progress feedback ---
        fprintf('✅ Completed test %d: %s\n', i, name);
    end

    % --- Convert once at the end ---
    Results = cell2table(ResultsCell, ...
        'VariableNames',{'Test','HardBrakes','NegVel','NegGap','MinGap','MaxAccel'});

    disp(Results)
    writetable(Results,'svsim_results.csv');
    fprintf('\n✅ All simulations completed. Results saved to svsim_results.csv\n');
end

% =========================================================================
function data = get_data(sig)
    % Handle multiple possible Simulink signal formats safely
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
function t = get_time(sig)
    % Extracts the time vector safely from various signal types
    if isa(sig, 'timeseries')
        t = sig.Time;
    elseif istimetable(sig)
        t = sig.Time;
    elseif isstruct(sig) && isfield(sig, 'time')
        t = sig.time;
    else
        % fallback to simple index-based vector
        data = get_data(sig);
        t = (0:length(data)-1)';
    end
end
