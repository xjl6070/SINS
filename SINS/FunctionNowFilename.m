function [ fname ] = FunctionNowFilename( pre, post )
    if nargin == 0
        pre = '';
        post = '';
    elseif nargin == 1
        post = '';
    end
    t = clock; % Get current time
    fname = [pre, num2str(t(1:1), '%04d'), ...  % Year
                  num2str(t(2:3), '%02d'), '_', ...   % -month-day_
                  num2str(t(4:5), '%02d'), ...  % hour min
                  num2str(fix(t(6)*1000), '%05d'), post]; % sec+ms
end