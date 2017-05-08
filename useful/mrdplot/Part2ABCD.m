[data, ~, ~, ~] = mrdplot_convert('d00059');
markerData=data(:,81:104);

pose=[-11 0 0];
init_vel=[0.02 -0.05 0.01];
quaternion=[1 0 0 0];

%....................................PART2A...............................................................%

%finding the origin and the axis of the coordinate frame
origin= (markerData(1,1:3))';

axispoint1=(markerData(1,4:6))'-origin;
axispoint1=axispoint1/norm(axispoint1);

axispoint2=(markerData(1,7:9))'-origin;
axispoint2=axispoint2/norm(axispoint2);

axispoint3=(markerData(1,13:15))'-origin;
axispoint3=axispoint3/norm(axispoint3);

init_orient=[axispoint1 axispoint2 axispoint3];

for n=2:10000
    %finding the changing position at each instance
     pose = [pose; pose(n-1,:)+0.01*init_vel];
    
    %finding rotation by subtracting the translational component
    origin=(markerData(n,1:3))';
    
    rotation1=(markerData(n,4:6))'-origin-0.01*init_vel';
    rotation1=rotation1/norm(rotation1);
    
    rotation2=(markerData(n,7:9))'-origin-0.01*init_vel';
    rotation2=rotation2/norm(rotation2);
    
    rotation3=(markerData(n,13:15))'-origin-0.01*init_vel';
    rotation3=rotation3/norm(rotation3);
    
    fin_orient=[rotation1 rotation2 rotation3];
    
    rotation=fin_orient/init_orient;
    quaternion=[quaternion;rotm2quat(rotation)];
   
end

comData=[pose quaternion];
save -ascii 'problem_2_0.dat' comData

%..........................................PART2B...........................................................%

zyx = quat2eul(quaternion(1,:));
init_angle = flip(zyx);
ang_vel = [0, 0, 0];

for n = 2:10000
   zyx = quat2eul(quaternion(n,:));
   next_angle = flip(zyx);
   w = (next_angle - init_angle)*100;
   
   init_angle = next_angle;
   ang_vel = [ang_vel; w(1) w(2) w(3)];
end

save -ascii 'problem_2_1.dat' ang_vel

%.......................................PART2C..............................................................%

ang_acc = [0, 0, 0; 0, 0, 0];

for n = 2:9999
    acc = (ang_vel(n,:)-ang_vel(n-1,:))*100;
    ang_acc = [ang_acc; acc];
end

save -ascii 'problem_2_2.dat' ang_acc

%...........................................PART2D..........................................................%
i = floor(linspace(1,10000));

wx = ang_vel(i,1);
wy = ang_vel(i,2);
wz = ang_vel(i,3);

wxdot = ang_acc(i,1);
wydot = ang_acc(i,2);
wzdot = ang_acc(i,3);

A1 = [wxdot(1) wydot(1)-(wx(1)*wz(1)) wzdot(1)+(wx(1)*wy(1)) wy(1)*wz(1) wy(1)^2-wz(1)^2 wy(1)*wz(1)];
A2 = [wx(1)*wz(1) wxdot(1)+(wy(1)*wz(1)) wx(1)^2+wz(1)^2 wydot(1) wzdot(1)-(wx(1)*wy(1)) wx(1)*wz(1)];
A3 = [wx(1)*wy(1) wx(1)^2-wy(1)^2 wxdot(1)-(wy(1)*wz(1)) -(wx(1)*wy(1)) wydot(1)+(wx(1)*wz(1)) wzdot(1)];

A = [A1; A2; A3];

for n = 2:100
    A1 = [wxdot(n) wydot(n)-(wx(n)*wz(n)) wzdot(n)+(wx(n)*wy(n)) wy(n)*wz(n) wy(n)^2-wz(n)^2 wy(n)*wz(n)];
    A2 = [wx(n)*wz(n) wxdot(n)+(wy(n)*wz(n)) wx(n)^2+wz(n)^2 wydot(n) wzdot(n)-(wx(n)*wy(n)) wx(n)*wz(n)];
    A3 = [wx(n)*wy(n) wx(n)^2-wy(n)^2 wxdot(n)-(wy(n)*wz(n)) -(wx(n)*wy(n)) wydot(n)+(wx(n)*wz(n)) wzdot(n)];

    A = [A; A1; A2; A3];   
end

[~,~,V] = svd(A);

moment = abs([V(1,6), V(2,6), V(3,6); V(3,6), V(4,6), V(5,6); V(3,6), V(5,6), V(6,6)]);

pWorld = [1 0 0; 0 1 0; 0 0 1];
inert_Rot = pWorld/init_orient;
inert_Quat = rotm2quat(inert_Rot);