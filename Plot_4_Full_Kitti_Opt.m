load('Test_4_Res.mat');

%Setup folders
addAll();

numDone = 100;

%convert from position to error
actualR = zeros(6,3);
actualR(2,:) = [-0.0148243143349357,0.00203019194952995,0.000770383764607292];
actualR(3,:) = [1.21437851395341,-1.20575830275602,1.20140448755769];
actualR(4,:) = [1.19262407657219,-1.24394315617111,1.20667889127063];
actualR(5,:) = [1.21927558122103,-1.20959073025790,1.20612899666033];
actualR(6,:) = [1.19258227358849,-1.23385869118214,1.20595334577645];

actualT = zeros(6,3);
actualT(2,:) = [0.810543972031926,-0.307054372250861,0.802723994558354];
actualT(3,:) = [-0.0137776900000000,-0.0554211700000000,-0.291858900000000];
actualT(4,:) = [-0.542627593414459,-0.0466888339330226,-0.305387140612605];
actualT(5,:) = [0.0472557452439009,-0.0549987883746558,-0.288566226572909];
actualT(6,:) = [-0.481361260988264,-0.0484258599349219,-0.295491738376115];

Rdiff = zeros(6,3,numDone);
Tdiff = zeros(6,3,numDone);

numSD = zeros(6,6,numDone);

for i = 1:numDone
    Rdiff(:,:,i) = outT{i}(:,4:6) - actualR;
    Tdiff(:,:,i) = outT{i}(:,1:3) - actualT;
    
    numSD(:,:,i) = [abs(Tdiff(:,:,i)), abs(Rdiff(:,:,i))] ./ sqrt(outV{i});
end

ROut = zeros(numDone,6,3);
TOut = zeros(numDone,6,3);

%convert to angular error in degrees
for i = 1:size(Rdiff,1)
    for j = 1:size(Rdiff,3)
        [r,p,y] = dcm2angle(vec2rot(Rdiff(i,:,j)'));
        ROut(j,i,1) = abs(r)*180/pi;
        TOut(j,i,1) = abs(Tdiff(i,1,j));
        ROut(j,i,2) = abs(p)*180/pi;
        TOut(j,i,2) = abs(Tdiff(i,2,j));
        ROut(j,i,3) = abs(y)*180/pi;
        TOut(j,i,3) = abs(Tdiff(i,3,j));
    end
end

%ditch first column
ROut = ROut(:,2:end,:);
TOut = TOut(:,2:end,:);


%% plot
close all
figure
hold on;

labels = {'IMU  ', 'Cam0 ', 'Cam1 ', 'Cam2 ', 'Cam3 '};
labels = repmat(labels, size(Rdiff,3),1);
labels = cell2mat(labels(:));
blanks = {'_    ','__   ', '___  ', '____ ', '_____'};
blanks = repmat(blanks, size(Rdiff,3),1);
blanks = cell2mat(blanks(:));

subplot(3,1,1);
boxplot(ROut(:,:,1),blanks);
set(gca,'OuterPosition',[0 0.62 1 0.36]);
ylabel('Roll');
subplot(3,1,2);
boxplot(ROut(:,:,2),blanks);
set(gca,'OuterPosition',[0 0.31 1 0.36]);
ylabel('Pitch');
subplot(3,1,3);
boxplot(ROut(:,:,3),labels);
set(gca,'OuterPosition',[0 0 1 0.36]);
ylabel('Yaw');
set(gcf,'color','w');

figure

subplot(3,1,1);
boxplot(TOut(:,:,1),blanks);
set(gca,'OuterPosition',[0 0.62 1 0.36]);
ylabel('X');
subplot(3,1,2);
boxplot(TOut(:,:,2),blanks);
set(gca,'OuterPosition',[0 0.31 1 0.36]);
ylabel('Y');
subplot(3,1,3);
boxplot(TOut(:,:,3),labels);
set(gca,'OuterPosition',[0 0 1 0.36]);
ylabel('Z');
set(gcf,'color','w');