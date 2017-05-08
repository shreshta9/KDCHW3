%%% criterion for optimizing initial center of mass location and velocity
function score=criterion(p)
[data, ~, ~, ~] = mrdplot_convert('d00059');

i = linspace(1,10000);
markerData = [i' data(i, 81:104)];

save -ascii 'markerData' markerData
global markerData
global NF

NM = 8; % number of markers
dt = 0.01;

% pull out parameters
com = [ p(1) p(2) p(3) ];
vel = [ p(4) p(5) p(6) ];

score = 0;

% calculate distances from com to each marker.
d2(NM) = 0;
for j = 1:NM
 % last +1 to skip initial count variable
 v = markerData(1,(3*(j-1)+1+1):(3*(j-1)+3+1)) - com;
 d2(j) = v*v'; % this is actually an inner product (since v is horizontal)
end

d2;

for i = 2:NF
 for j = 1:NM
  % last +1 to skip initial count variable
  v = markerData(i,(3*(j-1)+1+1):(3*(j-1)+3+1)) - com - vel*dt*markerData(i,1);
  dist = v*v';
  score = score + (d2(j) - dist)*(d2(j) - dist);
 end
end

end
