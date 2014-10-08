function [ camData ] = genKittiCam( kittiPath, plotCam, range, idx )
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
        camData.folder = [kittiPath 'image_00\data\'];
    case 2
        camData.folder = [kittiPath 'image_01\data\'];
    case 3
        camData.folder = [kittiPath 'image_02\data\'];
    case 4
        camData.folder = [kittiPath 'image_03\data\'];
    otherwise
end

%intrinsic camera parameters
switch idx
    case 1
        camData.K = [979.92,0,690,0;0,974.1183,248.6443,0;0,0,1,0];
        camData.D = [-3.745594e-01 2.049385e-01 1.110145e-03 1.379375e-03 -7.084798e-02];
    case 2
        camData.K = [990.3522,0,702,0;0,985.5674,260.7319,0;0,0,1,0];
        camData.D = [-3.712084e-01 1.978723e-01 -3.709831e-05 -3.440494e-04 -6.724045e-02];
    case 3
        camData.K = [960.1149,0,694.7923,0;0,954.8911,240.3547,0;0,0,1,0];
        camData.D = [-3.685917e-01 1.928022e-01 4.069233e-04 7.247536e-04 -6.276909e-02];
    case 4
        camData.K = [904.9931,0,695.7698,0;0,900.4945,238.982,0;0,0,1,0];
        camData.D = [-3.735725e-01 2.066816e-01 -6.133284e-04 -1.193269e-04 -7.600861e-02];
    otherwise
end

if(isempty(range))
   range = 1:size(dir([camData.folder,'*.png']));
end

    %get range of data
    camData.files = dir([camData.folder,'*.png']);
    camData.files = camData.files(range);

    %preallocate memory
    camData.T_Skm1_Sk = zeros(size(camData.files(:),1),6);
    camData.T_S1_Sk = zeros(size(camData.files(:),1),6);
    
    camData.T_Cov_Skm1_Sk = zeros(size(camData.files(:),1),6);
    camData.T_Cov_Skm1_Sk(1,:) = inf(1,6);
    
    camData.time = ReadKittiTimestamps([camData.folder,'..\']);
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
    image = uint8(undistort(double(image), camData.D, camData.K(1:3,1:3)));

    for frame = 2:size(camData.files,1)

        %% get inital tform
        
        fprintf('Finding Transform for image %i of %i for camera %i\n', frame,size(camData.files,1),idx);

        imageOld = image;
        %read new data
        image = imread([camData.folder camData.files(frame).name]); 
        if(size(image,3) == 3)
            image = rgb2gray(image);
        end
        image = uint8(undistort(double(image), camData.D, camData.K(1:3,1:3)));

        %find transformation
        try
            [camData.T_Skm1_Sk(frame,:), camData.T_Cov_Skm1_Sk(frame,:)] = getTcam(image, imageOld, [], ones(size(image)), camData.K,50000);
        catch
            camData.T_Skm1_Sk(frame,:) = [0,0,0,0,0,0,0];
            camData.T_Cov_Skm1_Sk(frame,:) = 1000*ones(1,6);
        end
        
        %generate absolute transformations
        camData.T_S1_Sk(frame,:) = tran2vec(vec2tran(camData.T_S1_Sk(frame-1,:)')*vec2tran(camData.T_Skm1_Sk(frame,:)'))';
    
    end
    
end
        
        
        
        
        