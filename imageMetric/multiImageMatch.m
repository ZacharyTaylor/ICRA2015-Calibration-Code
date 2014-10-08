function [ T, T_Var ] = multiImageMatch( camB, camA )
matchNum = 50;
dataNum = datasample(1:size(camA.files,1),matchNum,'Replace',false);

pointsA = cell(matchNum,1);
pointsB = cell(matchNum,1);

for i = 1:matchNum
    
    image = imread([camA.folder camA.files(dataNum(i)).name]);
    if(size(image,3) == 3)
        image = rgb2gray(image);
    end
    image = uint8(undistort(double(image), camA.D, camA.K(1:3,1:3)));
    surfA = detectSURFFeatures(image);
    [featA, surfA] = extractFeatures(image, surfA);

    image = imread([camB.folder camB.files(dataNum(i)).name]);
    if(size(image,3) == 3)
        image = rgb2gray(image);
    end
    image = uint8(undistort(double(image), camB.D, camB.K(1:3,1:3)));
    surfB = detectSURFFeatures(image);
    [featB, surfB] = extractFeatures(image, surfB);
    
    matches = matchFeatures(featA, featB, 'Prenormalized', true) ;

    pointsA{i} = surfA.Location(matches(:, 1),:);
    pointsB{i} = surfB.Location(matches(:, 2),:);
end

pointsA = cell2mat(pointsA);
pointsB = cell2mat(pointsB);

[T, T_Var] = getTandPoints(pointsA, pointsB, camA.K, camB.K);
end

function [T_Ckm1_Ck, T_Cov_Ckm1_Ck] = getTandPoints(pointsA, pointsB, KA, KB)

    %get fundemental matrix
    [F, inliers] = estimateFundamentalMatrix(pointsA,pointsB,'NumTrials',500);
    pointsA = pointsA(inliers,:);
    pointsB = pointsB(inliers,:);
    
    %get essential matrix
    E = KB(1:3,1:3)'*F*KA(1:3,1:3);

    %Hartley matrices
    W = [0 -1 0; 1 0 0; 0 0 1];
       
    %get P
    [U,~,V] = svd(E);
    RA = U*W*V';
    RB = U*W'*V';
    
    RA = RA*sign(RA(1,1));
    RB = RB*sign(RB(1,1));
    
    TA = U(:,3);
    TB = -U(:,3);
    
    %get Transform
    T_Ckm1_Ck = zeros(4,4,4);
    
    T_Ckm1_Ck(:,:,1) = [[RA,TA];[0,0,0,1]];
    T_Ckm1_Ck(:,:,2) = [[RA,TB];[0,0,0,1]];
    T_Ckm1_Ck(:,:,3) = [[RB,TA];[0,0,0,1]];
    T_Ckm1_Ck(:,:,4) = [[RB,TB];[0,0,0,1]];
        
    best = zeros(4,1);
 
    %match points
    points = zeros(size(pointsA,1),6,4);
    for j = 1:4
        T_Ckm1_Ck(:,:,j) = T_Ckm1_Ck(:,:,j);
        points(:,:,j) = getPoints(pointsA,pointsB,T_Ckm1_Ck(:,:,j),KA,KB);
        best(j,1) = median(points(:,3,j));% + median(points(:,6,j)));
    end
    
    [~,idx] = max(best);
    T_Ckm1_Ck = T_Ckm1_Ck(:,:,idx);
    
    %bootstrap data
    T_Cov_Ckm1_Ck = zeros(100,6);
    for i = 1:100
        [pBBS,idx] = datasample(pointsB,size(pointsB,1));
        pABS = pointsA(idx,:);
        F = estimateFundamentalMatrix(pABS,pBBS,'Method','Norm8Point');
    
        %get essential matrix
        E = KB(1:3,1:3)'*F*KA(1:3,1:3);
       
        %get P
        [U,~,V] = svd(E);
        RA = U*W*V';
        RB = U*W'*V';
        
        if (sum(U(:,3)) > 0)
            T = U(:,3);
        else
            T = -U(:,3);
        end
        
        if (sum(diag(RA)) > sum(diag(RB)))
            T_Cov_Ckm1_Ck(i,:) = tran2vec([RA,T;[0,0,0,1]]);
        else
            T_Cov_Ckm1_Ck(i,:) = tran2vec([RB,T;[0,0,0,1]]);
        end
    end
    T_Cov_Ckm1_Ck = var(T_Cov_Ckm1_Ck)';
    T_Ckm1_Ck = tran2vec(T_Ckm1_Ck)';
end

function [points] = getPoints(pointsA,pointsB,T,KA,KB)
    
    %match points
    points = zeros(size(pointsA,1),6);
    
    P1 = KA*T;
    P2 = KB;
    for i = 1:size(pointsA,1)

        A = zeros(4,4);
        A(1,:) = pointsA(i,1)*P1(3,:)' - P1(1,:)';
        A(2,:) = pointsA(i,2)*P1(3,:)' - P1(2,:)';
        A(3,:) = pointsB(i,1)*P2(3,:)' - P2(1,:)';
        A(4,:) = pointsB(i,2)*P2(3,:)' - P2(2,:)';

        [~, ~, V] = svd(A);
        points(i,1:3) = V(1:3,4)'/V(4,4);
        %temp = T*[points(i,1:3),1]';
        %points(i,4:6) = temp(1:3);
    end
end

