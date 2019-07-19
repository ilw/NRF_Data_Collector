%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BLE PPG data reading 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fid = fopen('PPG_BLE_Data.bin','r');
Number_of_LEDS = 2;
dataLength = 17*Number_of_LEDS*3;  %3 bytes per LED and then 17 samples per block
timeKeywordSize = 4;
timeSize = 4;
timestep = 1/31250;


%get file size
fseek(fid,0,1);
fsize = ftell(fid);
frewind(fid);

%go to 4 bytes from the beginning (i.e. skip the time keyword)
fseek(fid,4,-1);
ppgTs = fread(fid,'uint32',dataLength+timeKeywordSize);


dPpgTs = diff(ppgTs);
figure;
subplot(2,1,1); plot(ppgTs*timestep); title('PPG timestamps (should be a nice diagonal line)');
subplot(2,1,2); plot(dPpgTs*timestep);title('PPG differential timestamps ( should be ~flat line <10% variation)');

fseek(fid,8,-1); %8 bytes from beginning (skip time keyword and timestamp)

%Read the timestamps which occur at fixed spacing in the file
% (spacing depends on number of LEDs active)
if Number_of_LEDS == 1
    ppgData = fread(fid,'51*uint8',8);
elseif Number_of_LEDS ==2
    ppgData = fread(fid,'102*uint8',8);
else 
    ppgData = fread(fid,'153*uint8',8);
end

%trim off data that isn't a whole sample
ppgData = ppgData(1:(end-mod(length(ppgData),3))); 

d1 = bitshift(ppgData(1:3:end),16);
d2 = bitshift(ppgData(2:3:end),8);
d3 = ppgData(3:3:end);

d = d1+d2+d3;


red = d(1:2:end);
ir = d(2:2:end);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


ppgStartTime = ppgTs(1)-round(mean(diff(ppgTs)));
%Following line had to be changed to cope with incomplete batches of samples...
ppgTimeStampVector = interp1(1:17:(17*(ceil(length(red)/17)))+1, [ppgStartTime; ppgTs], 1:length(red));
ppgTimeStampVectorSeconds = ppgTimeStampVector /31250;

ppgSamplingRate = length(red)/(ppgTimeStampVectorSeconds(end) - ppgTimeStampVectorSeconds(1));

Recording_Length = length(red)/ppgSamplingRate;




