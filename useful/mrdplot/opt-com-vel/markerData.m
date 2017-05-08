[data, ~, ~, ~] = mrdplot_convert('d00059');

i = linspace(1,10000);
markerData = [i' data(i, 81:104)];

save -ascii 'markerData' markerData