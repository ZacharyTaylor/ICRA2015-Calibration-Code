load('Test_2_Res.mat');

%Setup folders
addAll();

%convert from position to error
actualR = zeros(6,3);
actualR(2,:) = [-1.23012360398877,1.18821891665826,1.18912896358993];
actualR(3,:) = [-1.56167478396818,0.229046415071725,0.214271556488645];
actualR(4,:) = [-1.43393422167173,-0.747245814613889,-0.768115419482089];
actualR(5,:) = [-0.817941887443196,-1.66662309175452,-1.68142301299025];
actualR(6,:) = [-0.365584626847446,2.03202234053439,2.04550939637492];
%actualR(7,:) = [2.24603963878014,-2.18734052673417,0.0137927830930595];
actualR = repmat(actualR,[1,1,size(RErr,3)]);

actualT = zeros(6,3);
actualT(2,:) = [-0.000816102168693086,0.368709507210727,-0.0253489088941838];
actualT(3,:) = [-0.0149314506916069,0.368622116010178,-0.0342986241339099];
actualT(4,:) = [-0.0109890425556882,0.368670255578844,-0.0524753140852584];
actualT(5,:) = [0.00674778612411340,0.368642755135233,-0.0552672200028299];
actualT(6,:) = [0.0119563010802032,0.368875454597967,-0.0372075202988963];
%actualT(7,:) = [0.00290204042329266,0.0132375080852727,-0.430620574697328];
actualT = repmat(actualT,[1,1,size(TErr,3)]);

Rdiff = zeros(size(RErr,3),3,15);
Tdiff = zeros(size(RErr,3),3,15);

RdiffS = zeros(size(RErr,3),3,15);
TdiffS = zeros(size(RErr,3),3,15);

for i = 1:size(RErr,3)
    m = 0;
    for j = 1:6
        for k = 1:6
            if k > j
                m = m+1;
                tempA = vec2tran([TErr(j,:,i)';RErr(j,:,i)']) / vec2tran([TErr(k,:,i)';RErr(k,:,i)']);
                tempB = vec2tran([actualT(j,:,i)';actualR(j,:,i)']) / vec2tran([actualT(k,:,i)';actualR(k,:,i)']);
                temp = tempA/tempB;
                temp = tran2vec(temp);
                Tdiff(i,:,m) = temp(1:3);
                Rdiff(i,:,m) = temp(4:6);
                
                tempA = vec2tran([TErrSep(j,:,i)';RErrSep(j,:,i)']) / vec2tran([TErrSep(k,:,i)';RErrSep(k,:,i)']);
                tempB = vec2tran([actualT(j,:,i)';actualR(j,:,i)']) / vec2tran([actualT(k,:,i)';actualR(k,:,i)']);
                temp = tempA/tempB;
                temp = tran2vec(temp);
                TdiffS(i,:,m) = temp(1:3);
                RdiffS(i,:,m) = temp(4:6);
            end
        end
    end
end

%convert to angular error in degrees and variance to std
for i = 1:size(Rdiff,1)
    for j = 1:size(Rdiff,3)
        [r,p,y] = dcm2angle(vec2rot(Rdiff(i,:,j)'));
        Rdiff(i,:,j) = abs([r,p,y])*180/pi;
        [r,p,y] = dcm2angle(vec2rot(RdiffS(i,:,j)'));
        RdiffS(i,:,j) = abs([r,p,y])*180/pi;
    end
end

Rdiff = mean(Rdiff,3);
RdiffS = mean(RdiffS,3);

Tdiff = mean(Tdiff,3);
TdiffS = mean(TdiffS,3);


%% plot
close all
figure
hold on;

blanks = {'_     ','__    '};
blanks = repmat(blanks, size(Rdiff,1),1);
blanks = cell2mat(blanks(:));
labels = {'Combined', 'Seperate'};
labels = repmat(labels, size(Rdiff,1),1);
labels = cell2mat(labels(:));

subplot(3,1,1);
boxplot([abs(Rdiff(:,1)),abs(RdiffS(:,1))],blanks);
set(gca,'OuterPosition',[0 0.62 1 0.36]);
ylabel('Roll');
subplot(3,1,2);
boxplot([abs(Rdiff(:,2)),abs(RdiffS(:,2))],blanks);
set(gca,'OuterPosition',[0 0.31 1 0.36]);
ylabel('Pitch');
subplot(3,1,3);
boxplot([abs(Rdiff(:,3)),abs(RdiffS(:,3))],labels);
set(gca,'OuterPosition',[0 0 1 0.36]);
ylabel('Yaw');
set(gcf,'color','w');

figure

subplot(3,1,1);
boxplot([abs(Tdiff(:,1)),abs(TdiffS(:,1))],blanks);
set(gca,'OuterPosition',[0 0.62 1 0.36]);
ylabel('X');
subplot(3,1,2);
boxplot([abs(Tdiff(:,2)),abs(TdiffS(:,2))],blanks);
set(gca,'OuterPosition',[0 0.31 1 0.36]);
ylabel('Y');
subplot(3,1,3);
boxplot([abs(Tdiff(:,3)),abs(TdiffS(:,3))],labels);
set(gca,'OuterPosition',[0 0 1 0.36]);
ylabel('Z');
set(gcf,'color','w');