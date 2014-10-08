function [ pos, speed, time, error ] = ReadNavData( fileName )
%READVELDATA Reads binary nav data

blockSize = 100000;

fid = fopen(fileName,'r');

time = []; pos = []; speed = []; error = [];

run = true;

%read a block of data
idx = 0;
while run
    fseek(fid,116*blockSize*idx,'bof');
    t = fread(fid, blockSize, '*uint64', 104);
    fseek(fid,8+116*blockSize*idx,'bof');
    
    x = fread(fid, blockSize, '*double', 104);
    fseek(fid,16+116*blockSize*idx,'bof');
    y = fread(fid, blockSize, '*double', 104);
    fseek(fid,24+116*blockSize*idx,'bof');
    z = fread(fid, blockSize, '*double', 104);
    fseek(fid,32+116*blockSize*idx,'bof');
    
    rx = fread(fid, blockSize, '*double', 104);
    fseek(fid,40+116*blockSize*idx,'bof');
    ry = fread(fid, blockSize, '*double', 104);
    fseek(fid,48+116*blockSize*idx,'bof');
    rz = fread(fid, blockSize, '*double', 104);
    fseek(fid,56+116*blockSize*idx,'bof');
    
    vx = fread(fid, blockSize, '*double', 104);
    fseek(fid,64+116*blockSize*idx,'bof');
    vy = fread(fid, blockSize, '*double', 104);
    fseek(fid,72+116*blockSize*idx,'bof');
    vz = fread(fid, blockSize, '*double', 104);
    fseek(fid,80+116*blockSize*idx,'bof');
    
    wx = fread(fid, blockSize, '*double', 104);
    fseek(fid,88+116*blockSize*idx,'bof');
    wy = fread(fid, blockSize, '*double', 104);
    fseek(fid,96+116*blockSize*idx,'bof');
    wz = fread(fid, blockSize, '*double', 104);
    
    fseek(fid,104+116*blockSize*idx,'bof');
    e = fread(fid, blockSize, '*double', 104);
    
    if(feof(fid))
        run = false;
    end
    
    idx = idx +1;
    
    l = min([length(x),length(y),length(z),length(rx),length(ry),length(rz), length(vx), length(vy), length(vz), length(wx), length(wy), length(wz), length(t), length(e)]);
    
    t = t(1:l);
    
    x = x(1:l);
    y = y(1:l);
    z = z(1:l);
    
    rx = rx(1:l);
    ry = ry(1:l);
    rz = rz(1:l);
    
    vx = vx(1:l);
    vy = vy(1:l);
    vz = vz(1:l);
    
    wx = wx(1:l);
    wy = wy(1:l);
    wz = wz(1:l);
    
    e = e(1:l);
    
    time = [time; t];
    l = min([length(x),length(y),length(z),length(rx),length(ry),length(rz), length(vx), length(vy), length(vz), length(wx), length(wy), length(wz), length(t)]);
    
    tempPos = [x y z rx ry rz];
    e = repmat(e,1,7);
    e(:,1:3) = 0.0001;
    e(:,4) = e(:,4).^2;
    e(:,4:6) = (0.03*pi/180).^2;
    for j = 1:l
        tempMat = eye(4); tempMat(1:3,1:3) = angle2dcm(tempPos(j,4),tempPos(j,5),tempPos(j,6),'XYZ'); tempMat(1:3,4) = tempMat(1:3,1:3)*tempPos(j,1:3)';
        tempPos(j,:) = tran2vec(tempMat)';
    end
    
    error = [error; e];
    
    pos = [pos,tempPos];
    
    
    speed = [speed ;vx vy vz wx wy wz];
    error = [error; e];
end

fclose(fid);

end

