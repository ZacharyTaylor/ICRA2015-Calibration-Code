load('Test_3_Res.mat');

%Setup folders
addAll();

numMatch = 100;

baseT = baseT(1:numMatch,:);
levT = levT(1:numMatch,:);
gomT = gomT(1:numMatch,:);
colT = colT(1:numMatch,:);

levTB = levTB(1:numMatch,:);
gomTB = gomTB(1:numMatch,:);
colTB = colTB(1:numMatch,:);

%convert from position to error
actualR = [1.21437851395341,-1.20575830275602,1.20140448755769];
actualR = repmat(actualR,numMatch,1);
actualT = [-0.0137776900000000,-0.0554211700000000,-0.291858900000000];
actualT = repmat(actualT,numMatch,1);

RErr = baseT(:,4:6) - actualR;
TErr = baseT(:,1:3) - actualT;

RGomErr = gomT(:,4:6) - actualR;
TGomErr = gomT(:,1:3) - actualT;

RLevErr = levT(:,4:6) - actualR;
TLevErr = levT(:,1:3) - actualT;

RMatchErr = colT(:,4:6) - actualR;
TMatchErr = colT(:,1:3) - actualT;

RGomBErr = gomTB(:,4:6) - actualR;
TGomBErr = gomTB(:,1:3) - actualT;

RLevBErr = levTB(:,4:6) - actualR;
TLevBErr = levTB(:,1:3) - actualT;

RMatchBErr = colTB(:,4:6) - actualR;
TMatchBErr = colTB(:,1:3) - actualT;

%convert to angular error in degrees and variance to std
for i = 1:size(RErr,1)
    [RErr(i,:), ~] = covVar2Deg(RErr(i,:), zeros(1,6));
    [RGomErr(i,:), ~] = covVar2Deg(RGomErr(i,:), zeros(1,6));
    [RLevErr(i,:), ~] = covVar2Deg(RLevErr(i,:), zeros(1,6));
    [RMatchErr(i,:), ~] = covVar2Deg(RMatchErr(i,:), zeros(1,6));
    [RGomBErr(i,:), ~] = covVar2Deg(RGomBErr(i,:), zeros(1,6));
    [RLevBErr(i,:), ~] = covVar2Deg(RLevBErr(i,:), zeros(1,6));
    [RMatchBErr(i,:), ~] = covVar2Deg(RMatchBErr(i,:), zeros(1,6));
end

%% plot
close all
figure
hold on;

blanks = {'_   ','__  ','___ ','____'};
blanks = repmat(blanks, size(RErr,1),1);
blanks = cell2mat(blanks(:));
labels = {'Starting', 'Levinson', 'GOM     ', 'Colour  '};
labels = repmat(labels, size(RErr,1),1);
labels = cell2mat(labels(:));

subplot(3,1,1);
boxplot([abs(RErr(:,1)),abs(RLevErr(:,1)),abs(RGomErr(:,1)),abs(RMatchErr(:,1))],blanks);
set(gca,'OuterPosition',[0 0.62 1 0.36]);
axis([0 5 0 2]);
%set(gca,'YScale','log')
ylabel('Roll');
subplot(3,1,2);
boxplot([abs(RErr(:,2)),abs(RLevErr(:,2)),abs(RGomErr(:,2)),abs(RMatchErr(:,2))],blanks);
set(gca,'OuterPosition',[0 0.31 1 0.36]);
axis([0 5 0 2]);
%set(gca,'YScale','log')
ylabel('Pitch');
subplot(3,1,3);
boxplot([abs(RErr(:,3)),abs(RLevErr(:,3)),abs(RGomErr(:,3)),abs(RMatchErr(:,3))],labels);
set(gca,'OuterPosition',[0 0 1 0.36]);
axis([0 5 0 2]);
%set(gca,'YScale','log')
ylabel('Yaw');
set(gcf,'color','w');

figure

subplot(3,1,1);
boxplot([abs(TErr(:,1)),abs(TLevErr(:,1)),abs(TGomErr(:,1)),abs(TMatchErr(:,1))],blanks);
set(gca,'OuterPosition',[0 0.62 1 0.36]);
axis([0 5 0 1]);
%set(gca,'YScale','log')
ylabel('X');
subplot(3,1,2);
boxplot([abs(TErr(:,2)),abs(TLevErr(:,2)),abs(TGomErr(:,2)),abs(TMatchErr(:,2))],blanks);
set(gca,'OuterPosition',[0 0.31 1 0.36]);
axis([0 5 0 1]);
%set(gca,'YScale','log')
ylabel('Y');
subplot(3,1,3);
boxplot([abs(TErr(:,3)),abs(TLevErr(:,3)),abs(TGomErr(:,3)),abs(TMatchErr(:,3))],labels);
set(gca,'OuterPosition',[0 0 1 0.36]);
axis([0 5 0 1]);
%set(gca,'YScale','log')
ylabel('Z');
set(gcf,'color','w');

figure

subplot(3,1,1);
boxplot([abs(RErr(:,1)),abs(RLevBErr(:,1)),abs(RGomBErr(:,1)),abs(RMatchBErr(:,1))],blanks);
set(gca,'OuterPosition',[0 0.62 1 0.36]);
%axis([0 5 0 2]);
%set(gca,'YScale','log')
ylabel('Roll');
subplot(3,1,2);
boxplot([abs(RErr(:,2)),abs(RLevBErr(:,2)),abs(RGomBErr(:,2)),abs(RMatchBErr(:,2))],blanks);
set(gca,'OuterPosition',[0 0.31 1 0.36]);
%axis([0 5 0 2]);
%set(gca,'YScale','log')
ylabel('Pitch');
subplot(3,1,3);
boxplot([abs(RErr(:,3)),abs(RLevBErr(:,3)),abs(RGomBErr(:,3)),abs(RMatchBErr(:,3))],labels);
set(gca,'OuterPosition',[0 0 1 0.36]);
%axis([0 5 0 2]);
%set(gca,'YScale','log')
ylabel('Yaw');
set(gcf,'color','w');

figure

subplot(3,1,1);
boxplot([abs(TErr(:,1)),abs(TLevBErr(:,1)),abs(TGomBErr(:,1)),abs(TMatchBErr(:,1))],blanks);
set(gca,'OuterPosition',[0 0.62 1 0.36]);
%axis([0 5 0 0.5]);
%set(gca,'YScale','log')
ylabel('X');
subplot(3,1,2);
boxplot([abs(TErr(:,2)),abs(TLevBErr(:,2)),abs(TGomBErr(:,2)),abs(TMatchBErr(:,2))],blanks);
set(gca,'OuterPosition',[0 0.31 1 0.36]);
%axis([0 5 0 0.5]);
%set(gca,'YScale','log')
ylabel('Y');
subplot(3,1,3);
boxplot([abs(TErr(:,3)),abs(TLevBErr(:,3)),abs(TGomBErr(:,3)),abs(TMatchBErr(:,3))],labels);
set(gca,'OuterPosition',[0 0 1 0.36]);
%axis([0 5 0 0.5]);
%set(gca,'YScale','log')
ylabel('Z');
set(gcf,'color','w');
