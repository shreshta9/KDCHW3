load problem_2_0.dat;

rotGlobal = [0 0 1 0; 1 0 0 11; 0 -1 0 4; 0 0 0 1];
rotQuat = [0.6479 -0.7563 -0.0883 -0.0201];

lander = zeros(1,7);

for n = 1:10000
    artifact_pose = [problem_2_0(n,1:3)'; 1];  
    world_pose = rotGlobal*artifact_pose;
    
    lander_pose = world_pose(1:3)';
    
    current_quat = problem_2_0(n,4:end);
    lander_rot = quatMult(current_quat, rotQuat);
    
    if n>1
        if abs(sum(lander_rot-lander(n-1,4:end))) > 0.5
            lander_rot = -lander_rot;
        end
    end
    
    lander_rot = lander_rot/norm(lander_rot);
    
    lander = [lander; lander_pose lander_rot];
end

lander = lander(2:end,:);

save -ascii 'problem_3.dat' lander