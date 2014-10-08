function [ navData ] = genShrimpNav( shrimpPath, plotVel, range )
%GENLADYBUG Generates transforms using icp and the velodyne

%setup

%setup help info
navData.help = ['navData stores the following information:\n'...
'folder- the folder containing the nav solution used\n'...
'files- the name of the nav files used to find the transforms\n'...
'help- this information...'...
'T_Skm1_Sk- the transformation from the frame of the sensor at timestep k-1 to its frame at timestep k\n'...
'T_S1_Sk- the transformation from the frame of the sensor at timestep 1 to its frame at timestep k\n'];

navData.folder = [shrimpPath '/Processed/novatel/'];
  
navData.files = dir([navData.folder,'*.bin']);

[pos,~,tnav,error] = ReadNavData( [navData.folder navData.files(1).name] );

if(isempty(range))
    range = (1:size(pos,1))';
end

pos = pos(range,:);
error = error(range,:);
navData.time = tnav(range);

navData.T_Skm1_Sk = zeros(size(pos,1),6);
navData.T_S1_Sk = zeros(size(pos,1),6);
navData.T_Skm1_Sk(1,:) = tran2vec(eye(4));
navData.T_Cov_Skm1_Sk = zeros(size(navData.files(:),1),6);
velData.T_Cov_Skm1_Sk(1,:) = inf(1,6);

navData.type = 'nav';
    
if(plotVel)
    figure;
    axis equal;
    hold on;
end

%setup loop
nav = vec2tran(pos(1,:)');
navData.T_Cov_Skm1_Sk(1,:) = error(1,:);

%find transform for each velodyne scan
for frame = 2:size(navData.T_Skm1_Sk,1)
    
    fprintf('Finding Transform for nav point %i of %i\n', frame-1, size(navData.T_Skm1_Sk,1));

    %store old data
    navOld = nav;

    %read new data
    nav = vec2tran(pos(frame,:)');
    
    %find sensor transforms
    try
        temp = nav/navOld;
    catch
        temp = eye(4);
    end
    
    navData.T_Skm1_Sk(frame,:) = tran2vec(temp)';
    navData.T_Cov_Skm1_Sk(frame,:) = error(frame,:);
    
    %not sure why reverse is needed will look into it
    navData.T_Skm1_Sk(frame,1:3) = -navData.T_Skm1_Sk(frame,1:3);

    %generate absolute transformations
    navData.T_S1_Sk(frame,:) = tran2vec(vec2tran(navData.T_S1_Sk(frame-1,:)')*vec2tran(navData.T_Skm1_Sk(frame,:)'))';
        
    if(plotVel)
        %plot points
        T = vec2tran(navData.T_S1_Sk(frame,:)');
        plot3(T(1,4),T(2,4),T(3,4));
        drawnow;
    end
end 