function [ navData ] = genKittiNav( kittiPath, plotVel, range )
%GENLADYBUG Generates transforms using icp and the velodyne

%setup

%setup help info
navData.help = ['navData stores the following information:\n'...
'folder- the folder containing the nav solution used\n'...
'files- the name of the nav files used to find the transforms\n'...
'help- this information...'...
'T_Skm1_Sk- the transformation from the frame of the sensor at timestep k-1 to its frame at timestep k\n'...
'T_S1_Sk- the transformation from the frame of the sensor at timestep 1 to its frame at timestep k\n'];

navData.folder = [kittiPath '/oxts/data/'];
  
navData.files = dir([navData.folder,'*.txt']);

if(isempty(range))
    range = 1:size(navData.files(:));
end

navData.files = navData.files(range);
navData.T_Skm1_Sk = zeros(size(navData.files(:),1),6);
navData.T_S1_Sk = zeros(size(navData.files(:),1),6);
navData.T_Skm1_Sk(1,:) = tran2vec(eye(4));
navData.T_Cov_Skm1_Sk = zeros(size(navData.files(:),1),6);
navData.T_Cov_Skm1_Sk(1,:) = inf(1,6);

navData.time = ReadKittiTimestamps([navData.folder '../']);

navData.type = 'nav';
    
if(plotVel)
    figure;
    axis equal;
    hold on;
end

%setup loop
[nav, navData.T_Cov_Skm1_Sk(1,:)] = readKittiNavWithError([navData.folder navData.files(1).name]);

%find transform for each velodyne scan
for frame = 2:size(navData.files,1)
    
    fprintf('Finding Transform for nav point %i of %i\n', frame-1, size(navData.files,1));

    %store old data
    navOld = nav;

    %read new data
    [nav, navData.T_Cov_Skm1_Sk(frame,:)] = readKittiNavWithError([navData.folder navData.files(frame).name]);
    %[temp, navData.T_Cov_Skm1_Sk(:,:,frame)] = readKittiNavDiff([navData.folder navData.files(frame).name],(navData.time(frame)-navData.time(frame-1)));
    
    %find sensor transforms
    try
        temp = inv(navOld\nav);
    catch
        temp = eye(4);
    end
    
    navData.T_Skm1_Sk(frame,:) = tran2vec(temp)';
    
    %generate absolute camera transformations
    navData.T_S1_Sk(frame,:) = tran2vec(vec2tran(navData.T_S1_Sk(frame-1,:)')*vec2tran(navData.T_Skm1_Sk(frame,:)'))';
        
    if(plotVel)
        %plot points
        T = vec2tran(navData.T_S1_Sk(frame,:)');
        plot3(T(1,4),T(2,4),T(3,4));
        drawnow;
    end
end 


