defaultProfile = parallel.defaultClusterProfile;
myCluster = parcluster(defaultProfile);
maxWorkers = myCluster.NumWorkers;
localRun(2) = min(feature('numcores'), maxWorkers);
disp(' ');
disp([num2str(localRun(2)) ' CPU cores were detected and will be allocated for parallel processing.']);
disp(' ');
parpool('local',localRun(2));
% matlabpool(localRun(2));

timeseries_viewer_GUI