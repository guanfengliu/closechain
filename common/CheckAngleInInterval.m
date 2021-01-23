function in_interval = CheckAngleInInterval(leftConfig, angle_intv)
%% this function is to check if a given angle leftConfig lies
%% in the interval represented by a 2D-vector angle_intv
%% Note: we have to leftConfig + 2pi  \in angle_intv, or
%% leftConfig - 2pi \in angle_intv
if (leftConfig <= angle_intv(2) & leftConfig >=angle_intv(1)) | (leftConfig <= angle_intv(2)+pi & leftConfig >=angle_intv(1)+pi)
    in_interval = true;
    return;
end
if (leftConfig + 2*pi <= angle_intv(2) & leftConfig + 2*pi >=angle_intv(1)) | (leftConfig + pi <= angle_intv(2) & leftConfig + pi >=angle_intv(1))
    in_interval = true;
    return;
end
if (leftConfig - 2*pi <= angle_intv(2) & leftConfig - 2*pi >=angle_intv(1)) | (leftConfig - 3*pi <= angle_intv(2) & leftConfig - 3*pi >=angle_intv(1))
    in_interval = true;
    return;
end
in_interval = false;
return;