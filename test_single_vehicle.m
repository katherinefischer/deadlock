t = (0:0.1:50)';             % time vector
vlead = 20*ones(size(t));    % lead velocity = 20 m/s
xfront = 40 + cumtrapz(t,vlead); % lead starts 40 m ahead

vlead_ts = timeseries(vlead,t);
xfront_ts = timeseries(xfront,t);

assignin('base','vlead_ts',vlead_ts);
assignin('base','xfront_ts',xfront_ts);
