function [xfront_ts, vlead_ts, name] = single_vehicle_scenario(testNum, t)
% SINGLE_VEHICLE_SCENARIO
% -------------------------------------------------------------------------
% Generates lead-vehicle position and velocity profiles for 10 single-vehicle tests.
% Each test stresses a different aspect of the ego controller.
%
% Outputs:
%   xfront_ts  - timeseries of lead position (m)
%   vlead_ts   - timeseries of lead velocity (m/s)
%   name       - string label for the test
%
% Compatible with: run_svsim_tests.m

x0 = 40;   % initial spacing (m)
v0 = 20;   % baseline lead speed (m/s)

switch testNum
    %------------------------------------------------------
    case 1
        name  = "steady";
        vlead = v0 * ones(size(t));                   % constant 20 m/s
        xfront = x0 + cumtrapz(t, vlead);

    %------------------------------------------------------
    case 2
        name  = "lead_brake";
        vlead = v0 * ones(size(t));
        % Gradual deceleration 20→10 m/s between 5–10 s
        idx = (t >= 5 & t < 10);
        vlead(idx) = v0 - 2*(t(idx)-5);
        vlead(t >= 10) = 10;
        xfront = x0 + cumtrapz(t, vlead);

    %------------------------------------------------------
    case 3
        name  = "lead_accel";
        vlead = v0 * ones(size(t));
        % Gradual acceleration 20→30 m/s between 5–10 s
        idx = (t >= 5 & t < 10);
        vlead(idx) = v0 + 2*(t(idx)-5);
        vlead(t >= 10) = 30;
        xfront = x0 + cumtrapz(t, vlead);

    %------------------------------------------------------
    case 4
        name  = "stationary_obstacle";
        vlead = v0 * ones(size(t));
        % Smooth stop 20→0 m/s between 7–10 s, then remain stopped
        idx = (t >= 7 & t < 10);
        vlead(idx) = v0 - (v0/3)*(t(idx)-7);
        vlead(t >= 10) = 0;
        xfront = x0 + cumtrapz(t, vlead);

    %------------------------------------------------------
    case 5
        name  = "cutin_close";
        x0    = 10;                                  % car appears close
        vlead = v0 * ones(size(t));                  % same speed as ego
        xfront = x0 + cumtrapz(t, vlead);

    %------------------------------------------------------
    case 6
        name  = "cutout";
        vlead = v0 * ones(size(t));
        % Accelerates away quickly after 15 s
        idx = (t >= 15);
        vlead(idx) = v0 + 10*(t(idx)-15);
        xfront = x0 + cumtrapz(t, vlead);

    %------------------------------------------------------
    case 7
        name  = "vary_timegap";
        % Lead oscillates gently around 20 ±3 m/s
        vlead = v0 + 3*sin(0.1*t);
        xfront = x0 + cumtrapz(t, vlead);

    %------------------------------------------------------
    case 8
        name  = "noisy_gap";
        % Measurement noise ±2 m
        vlead = v0 * ones(size(t));
        xnoise = 2*randn(size(t));
        xfront = x0 + cumtrapz(t, vlead) + xnoise;

    %------------------------------------------------------
    case 9
        name  = "stop_and_go";
        vlead = v0 * ones(size(t));
        % Smooth stop 20→0 between 7–10 s, hold, then restart 0→20 between 15–18 s
        idx1 = (t >= 7 & t < 10);
        vlead(idx1) = v0 - (v0/3)*(t(idx1)-7);
        vlead(t >= 10 & t < 15) = 0;
        idx2 = (t >= 15 & t < 18);
        vlead(idx2) = (v0/3)*(t(idx2)-15);
        vlead(t >= 18) = v0;
        xfront = x0 + cumtrapz(t, vlead);

    %------------------------------------------------------
    case 10
        name  = "gain_sensitivity";
        % Baseline motion, use different gains in controller for this test
        vlead = v0 * ones(size(t));
        xfront = x0 + cumtrapz(t, vlead);
end

% -------------------------------------------------------------------------
% Convert to timeseries for Simulink
xfront_ts = timeseries(xfront, t);
vlead_ts  = timeseries(vlead, t);

end
