fid = fopen('capture.bin','rb');
eegFid = fopen('EEG_BLE_Data.bin','wb');
ppgFid = fopen('PPG_BLE_Data.bin','wb');

eegKeyword='EEG_';
ppgKeyword='PPG_';
timeKeyword='Time';
keywordSize = 4;
% The data is sent from the dongle to the PC in USB packets of size 2048 bytes
% The packets are dumped to a file as binary.
% Each packet contains just EEG or just PPG data and is prefixed with a
% keyword ('EEG_' or 'PPG_') that identifies where the data comes from
% so each packet is a 4 byte keyword and 2044 bytes of data
packetSize = 2048; 

%GEt the size of the file in bytes
fseek(fid,0,1);
fsize = ftell(fid);
frewind(fid);

%Get the number of packets
nPackets = fsize/packetSize;  

%Read in each packet, check the keyword and save the data accordingly
for i=1:nPackets
    fseek(fid,(i-1)*packetSize,-1);
    keyword = strcat(char(fread(fid,4,'uint8')))';
    a=[];
    a = fread(fid,packetSize-keywordSize,'uint8');
    if strcmp(keyword,eegKeyword)
        fwrite(eegFid,a,'uint8');
    elseif strcmp(keyword,ppgKeyword)
        fwrite(ppgFid,a,'uint8');
    else
        disp('error')
    end
    
end

fclose(eegFid);
fclose(ppgFid);
