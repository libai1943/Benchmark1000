function is_valid = IsCurSolValid()
is_valid = 0;
load opti_flag.txt
if (opti_flag == 0)
    return;
end
if (~IsCollisionFree())
    return;
end
is_valid = 1;
end