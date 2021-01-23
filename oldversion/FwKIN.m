function leftEnd=FwKIN(linklength,Config)
%% @brief: this is the fundamental function for calculating the forward kinematics of a serial kinematic chain
%% @linklength: a vector of link lengths of size 1 * DOF
%% @Config: a matrix of joint angles  DOF * 1
%% Author: Leon G.F. Liu  09/23/2019

if length(linklength) ~= length(Config)
    fprintf(1,'linklength should have same dimension as config \n');
    return;
end
x=sum(linklength .* cos(Config'));
y=sum(linklength .* sin(Config'));
leftEnd=[x,y]';
