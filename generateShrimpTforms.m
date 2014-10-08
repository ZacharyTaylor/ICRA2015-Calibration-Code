% this script generates all the required transforms for the kitti
% dataset

%% user set variables

%path to data
kittiPath = '/home/z/Documents/Datasets/Shrimp/around-shed';

%Sets if the sensor transforms will be plotted
plotTforms = false;

%Setup folders
addAll();

%% motion estimation

%do things in parrallel to save time
parfor i = 1:6
    switch i
        case 1
            shrimpVelData = genShrimpVel(shrimpPath, plotTforms, []);
            parsave('shrimpVelData.mat', shrimpVelData, 'velData');
        case 2
            shrimpCamData = genShrimpCam(shrimpPath, plotTforms, [], 1);
            parsave('shrimpCam1Data.mat', shrimpCamData, 'cam1Data');
        case 3
            shrimpCamData = genShrimpCam(shrimpPath, plotTforms, [], 2);
            parsave('shrimpCam2Data.mat', shrimpCamData, 'cam2Data');
        case 4
            shrimpCamData = genShrimpCam(shrimpPath, plotTforms, [], 3);
            parsave('shrimpCam3Data.mat', shrimpCamData, 'cam3Data');
        case 5
            shrimpCamData = genShrimpCam(shrimpPath, plotTforms, [], 4);
            parsave('shrimpCam4Data.mat', shrimpCamData, 'cam4Data');
        case 6
            shrimpCamData = genShrimpCam(shrimpPath, plotTforms, [], 5);
            parsave('shrimpCam5Data.mat', shrimpCamData, 'cam5Data');
        case 7
            shrimpNavData = genShrimpNav(shrimpPath, plotTforms, []);
            parsave('shrimpNavData.mat', shrimpNavData, 'navData');    
        otherwise
    end
end
    
delete(gcp);