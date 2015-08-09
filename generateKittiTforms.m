% this script generates all the required transforms for the kitti
% dataset

%% user set variables

%path to data
kittiPath = 'C:\Users\Zachary\Documents\Datasets\Kitti\2011_09_26_drive_0035_extract\';

%Sets if the sensor transforms will be plotted
plotTforms = false;

%Setup folders
addAll();

%% Run motion estimation

%do things in parrallel to save time
parfor i = 1:6
    switch i
        case 1
            kittiVelData = genKittiVel(kittiPath, plotTforms, []);
            parsave('kittiVelData.mat', kittiVelData, 'velData');
        case 2
            kittiCamData = genKittiCam(kittiPath, plotTforms, [], 2);
            parsave('kittiCam2Data.mat', kittiCamData, 'cam2Data');
        case 3
            kittiCamData = genKittiCam(kittiPath, plotTforms, [], 3);
            parsave('kittiCam3Data.mat', kittiCamData, 'cam3Data');
        case 4
            kittiCamData = genKittiCam(kittiPath, plotTforms, [], 4);
            parsave('kittiCam4Data.mat', kittiCamData, 'cam4Data');
        case 5
            kittiCamData = genKittiCam(kittiPath, plotTforms, [], 1);
            parsave('kittiCam1Data.mat', kittiCamData, 'cam1Data');
        case 6
            kittiNavData = genKittiNav(kittiPath, plotTforms, []);
            parsave('kittiNavData.mat', kittiNavData, 'navData');    
        otherwise
    end
end
    
delete(gcp);