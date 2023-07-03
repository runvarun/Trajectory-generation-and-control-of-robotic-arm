function [thetalistNext] ...
         = NextState(thetalist, dthetalist, dt, maxjointvel)

% Takes thetalist: n-vector of joint variables,
%       dthetalist: n-vector of joint rates,
%       dt: The timestep delta t.
%       maxjointvel : The maximum joint velocity magnitudes 
%                     pi rad/s for the first 3 joints, 2*pi rad/s for the
%                     last 3 joints

% Returns thetalistNext: Vector of joint variables after dt from first 
%                        order Euler integration,

%   If joint velocities exceed the maximum allowed velocities, the function
%   will limit it to the maximum

for i=1:size(thetalist)
    if(maxjointvel(i)<dthetalist(i))
            dthetalist(i)=dthetalist(i);
    end
end

thetalistNext = thetalist + dt * dthetalist;
end