function [ camData ] = genShrimpCam( shrimpPath, plotCam, range, idx )
%GENKITTICAM Generates transformation of cameras and 3d points for the 4
%cameras used by the kitti car

%setup help info
help = ['camData stores the following information:\n'...
'folder- the folder containing the images used\n'...
'files- the name of the image files used to find the transforms\n'...
'help- this information...'...
'points- a structure holding the following information:\n'...
'\tImages- a nx1 vector of the timpsteps k, when the point appears in images\n'...
'\tLocation- a nx2 matrix of the [x,y] location of the point in each image\n'...
'\tLocation3D- the 3d location of the point with respect to the frame of image 1\n'...
'T_Skm1_Sk- the transformation from the frame of the sensor at timestep k-1 to its frame at timestep k\n'...
'T_S1_Sk- the transformation from the frame of the sensor at timestep 1 to its frame at timestep k\n'];
camData.help = help;

%set destination folder
switch idx
    case 1
        camData.folder = [shrimpPath '/Processed/ladybug/camera_0/'];
    case 2
        camData.folder = [shrimpPath '/Processed/ladybug/camera_1/'];
    case 3
        camData.folder = [shrimpPath '/Processed/ladybug/camera_2/'];
    case 4
        camData.folder = [shrimpPath '/Processed/ladybug/camera_3/'];
    case 5
        camData.folder = [shrimpPath '/Processed/ladybug/camera_4/'];
    otherwise
end

%intrinsic camera parameters
switch idx
    case 1
        camData.K = [403.431 0 621 0; 0 403.431 811.810 0; 0 0 1 0];
        camData.D = [0 0 0 0 0];
    case 2
        camData.K = [410.810 0 623 0; 0 410.810 810.002 0; 0 0 1 0];
        camData.D = [0 0 0 0 0];
    case 3
        camData.K = [412.206 0 628 0; 0 412.206 810.771 0; 0 0 1 0];
        camData.D = [0 0 0 0 0];
    case 4
        camData.K = [409.433 0 609 0; 0 409.433 816.045 0; 0 0 1 0];
        camData.D = [0 0 0 0 0];
    case 5
        camData.K = [404.472 0 627 0; 0 404.472 826.407 0; 0 0 1 0];
        camData.D = [0 0 0 0 0];
    otherwise
end

%mask
switch idx
    case 1
        camData.mask = rgb2gray(imread([shrimpPath '/Processed/ladybug/masks/cam0.png'])) > 0;
    case 2
        camData.mask = rgb2gray(imread([shrimpPath '/Processed/ladybug/masks/cam1.png'])) > 0;
    case 3
        camData.mask = rgb2gray(imread([shrimpPath '/Processed/ladybug/masks/cam2.png'])) > 0;
    case 4
        camData.mask = rgb2gray(imread([shrimpPath '/Processed/ladybug/masks/cam3.png'])) > 0;
    case 5
        camData.mask = rgb2gray(imread([shrimpPath '/Processed/ladybug/masks/cam4.png'])) > 0;
    otherwise
end

if(isempty(range))
   range = 1:size(dir([camData.folder,'*.jpg']));
end

    %get range of data
    camData.files = dir([camData.folder,'*.jpg']);
    camData.files = camData.files(range);

    %preallocate memory
    camData.T_Skm1_Sk = zeros(size(camData.files(:),1),6);
    camData.T_S1_Sk = zeros(size(camData.files(:),1),6);
    
    camData.T_Cov_Skm1_Sk = zeros(size(camData.files(:),1),6);
    camData.T_Cov_Skm1_Sk(1,:) = inf(1,6);
    
    camData.time = readTimeData([shrimpPath '/Processed/ladybug/timestamps.bin']);
    camData.time = camData.time(range);
    
    camData.type = 'camera';

    if(plotCam)
        figure;
        axis equal;
        hold on;
    end
       
    %find transform for each image
    image = imread([camData.folder camData.files(1).name]); 
    if(size(image,3) == 3)
        image = rgb2gray(image);
    end

    for frame = 2:size(camData.files,1)

        %% get inital tform
        
        fprintf('Finding Transform for image %i of %i for camera %i\n', frame,size(camData.files,1),idx);

        imageOld = image;
        %read new data
        image = imread([camData.folder camData.files(frame).name]); 
        if(size(image,3) == 3)
            image = rgb2gray(image);
        end

        %find transformation
        try
            [camData.T_Skm1_Sk(frame,:), camData.T_Cov_Skm1_Sk(frame,:)] = getTcam(image, imageOld, [], camData.mask, camData.K,50000);
        catch
            camData.T_Skm1_Sk(frame,:) = [0,0,0,0,0,0];
            camData.T_Cov_Skm1_Sk(frame,:) = 1000*ones(1,6);
        end
        
        %generate absolute transformations
        camData.T_S1_Sk(frame,:) = tran2vec(vec2tran(camData.T_S1_Sk(frame-1,:)')*vec2tran(camData.T_Skm1_Sk(frame,:)'))';
    
    end
    
end
        
        
        
        
        