%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BLE EEG data reading 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fid = fopen('EEG_BLE_Data.bin','r');
dataLength = 6129;  %227*27
paddingLength = 207;
timeKeywordSize = 4;
timeSize = 4;
timestep = 1/31250;

%go to 4 bytes from the beginning (i.e. skip the time keyword)
fseek(fid,4,-1);
%Read the timestamps which occur at fixed spacing in the file
eegTs = fread(fid,'uint32',dataLength+timeKeywordSize+paddingLength);

dEegTs = diff(eegTs);
figure;
subplot(2,1,1); plot(eegTs*timestep); title('EEG timestamps (should be a nice diagonal line)');
subplot(2,1,2); plot(dEegTs*timestep);title('EEG differential timestamps ( should be ~flat line <10% variation)');

%Go to 8 bytes from beginning (skip time keyword and timestamp)
fseek(fid,timeKeywordSize+timeSize,-1); 
%Read in a block of data (227 samples, each of which is 27 bytes)
eegData = fread(fid,'6129*uint8',timeKeywordSize+timeSize+paddingLength);
%trim off data that isn't a whole sample
eegData = eegData(1:(end-mod(length(eegData),27))); 


eegData1 = eegData(1:3:end);
eegData2 = eegData(2:3:end);
eegData3 = eegData(3:3:end);

eegVals = uint32(eegData1 * 2^24 + eegData2 * 2^16 + eegData3*2^8);
a = typecast(eegVals,'int32');
b = double(a) *(2.4/(12*2^24)) / 2^8; %get rid of lsb's and convert to millivolts
c = reshape(b,9,length(b)/9);
eegChannelised = c;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Unchanged code
data=[];
chanCount =0;
for i=2:9
   if sum(eegChannelised(i,1:10))~=0
       chanCount =chanCount +1;
       data(:,chanCount) = eegChannelised(i,:);
   end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

eegStartTime = eegTs(1)-round(mean(diff(eegTs)));
%Following line had to be changed to cope with incomplete batches of samples...
eegTimeStampVector = interp1(1:227:(227*(ceil(length(eegChannelised(1,:))/227)))+1, [eegStartTime; eegTs], 1:length(eegChannelised(1,:)));
eegTimeStampVectorSeconds = eegTimeStampVector /31250;

eegSamplingRate = length(eegChannelised(1,:))/(eegTimeStampVectorSeconds(end) - eegTimeStampVectorSeconds(1));

Recording_Length = length(eegChannelised(1,:))/eegSamplingRate;




