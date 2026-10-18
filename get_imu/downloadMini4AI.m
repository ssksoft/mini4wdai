%% Mini4AI IMU stream downloader for MATLAB R2025b
% Requires MG24 firmware with CMD_STREAM_START = 6.
clear; clc;

global MINI4AI_RX
MINI4AI_RX = struct('packets',{{}},'done',false,'lastSeq',-1,'duplicates',0);

SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0";
CMD_UUID     = "12345678-1234-5678-1234-56789abcdef4";
STATUS_UUID  = "12345678-1234-5678-1234-56789abcdef5";
BULK_UUID    = "12345678-1234-5678-1234-56789abcdefc";

fprintf("Connecting...\n");
b = ble("Mini4AI-IMU");
cmdChar    = characteristic(b,SERVICE_UUID,CMD_UUID);
statusChar = characteristic(b,SERVICE_UUID,STATUS_UUID);
bulkChar   = characteristic(b,SERVICE_UUID,BULK_UUID);

status = double(read(statusChar));
fprintf("Status = %d\n",status);
if status == 1
    error("MG24 is still logging. Turn it over and wait for DUMP_READY first.");
elseif status == 0
    error("MG24 reported ERROR.");
end

% Metadata is still requested once.
write(cmdChar,uint8(1),"uint8");
pause(0.25);
meta = uint8(read(bulkChar));
if numel(meta) < 16 || leU16(meta,1) ~= hex2dec('FFFE')
    error("Metadata packet was not received.");
end
imuCount = double(leU16(meta,4));
overflowCount = double(leU16(meta,6));
sampleHz = double(leU32(meta,13));
protocolVersion = double(meta(10));
fprintf("Samples=%d, Overflow=%d, Rate=%d Hz, Protocol=%d\n", ...
    imuCount,overflowCount,sampleHz,protocolVersion);
if protocolVersion < 2
    error("MG24 is not running the streaming firmware (protocol version 2).");
end

% Subscribe BEFORE starting the stream. MATLAB's callback reads the oldest
% queued notification so packets are not intentionally flushed.
subscribe(bulkChar,"notification");
bulkChar.DataAvailableFcn = @mini4aiOnData;
cleanupObj = onCleanup(@()mini4aiCleanup(bulkChar)); %#ok<NASGU>

MINI4AI_RX.packets = cell(imuCount,1);
MINI4AI_RX.done = false;
MINI4AI_RX.lastSeq = -1;
MINI4AI_RX.duplicates = 0;

fprintf("Streaming log...\n");
t0 = tic;
write(cmdChar,uint8([6 0 0]),"uint8");

% Guard against an endless wait. This does not determine transfer speed.
timeoutSec = 180;
lastPrint = tic;
while ~MINI4AI_RX.done && toc(t0) < timeoutSec
    pause(0.02);
    drawnow limitrate;
    if toc(lastPrint) >= 1
        received = sum(~cellfun('isempty',MINI4AI_RX.packets));
        fprintf("Received %d / %d\n",received,imuCount);
        lastPrint = tic;
    end
end
elapsed = toc(t0);

bulkChar.DataAvailableFcn = [];
unsubscribe(bulkChar);

receivedMask = ~cellfun('isempty',MINI4AI_RX.packets);
receivedCount = sum(receivedMask);
fprintf("Received %d / %d in %.1f s\n",receivedCount,imuCount,elapsed);
if ~MINI4AI_RX.done
    warning("Terminator was not received before timeout.");
end
if receivedCount ~= imuCount
    missing = find(~receivedMask)-1;
    error("Missing %d packets. First missing sequence: %d. Reduce STREAM_BURST in MG24 firmware.", ...
        numel(missing),missing(1));
end

% Decode after reception, not inside the callback.
timeUs = zeros(imuCount,1);
gx = zeros(imuCount,1); gy = zeros(imuCount,1); gz = zeros(imuCount,1);
ax = zeros(imuCount,1); ay = zeros(imuCount,1); az = zeros(imuCount,1);
for k = 1:imuCount
    p = MINI4AI_RX.packets{k};
    timeUs(k) = double(leU32(p,4));
    gx(k) = double(leI16(p,8))/10;
    gy(k) = double(leI16(p,10))/10;
    gz(k) = double(leI16(p,12))/10;
    ax(k) = double(leI16(p,14))/1000;
    ay(k) = double(leI16(p,16))/1000;
    az(k) = double(leI16(p,18))/1000;
end

T = table(timeUs/1e6,gx,gy,gz,ax,ay,az, ...
    'VariableNames',{'time_s','gx_dps','gy_dps','gz_dps','ax_g','ay_g','az_g'});
filename = "Mini4AI_" + string(datetime('now','Format','yyyyMMdd_HHmmss')) + ".csv";
writetable(T,filename);
fprintf("Saved: %s\n",filename);

figure;
subplot(2,1,1); plot(T.time_s,[T.gx_dps T.gy_dps T.gz_dps]);
grid on; xlabel("Time [s]"); ylabel("Gyro [dps]"); legend("gx","gy","gz");
subplot(2,1,2); plot(T.time_s,[T.ax_g T.ay_g T.az_g]);
grid on; xlabel("Time [s]"); ylabel("Accel [g]"); legend("ax","ay","az");

function mini4aiOnData(src,~)
    global MINI4AI_RX
    % Drain every queued notification, oldest first.
    while src.NumBytesAvailable > 0
        p = uint8(read(src,'oldest'));
        if numel(p) < 2, continue; end
        marker = leU16(p,1);
        if marker == hex2dec('FFFF')
            MINI4AI_RX.done = true;
            return;
        end
        if numel(p) ~= 19 || p(3) ~= 1, continue; end
        seq = double(marker);
        idx = seq + 1;
        if idx >= 1 && idx <= numel(MINI4AI_RX.packets)
            if ~isempty(MINI4AI_RX.packets{idx})
                MINI4AI_RX.duplicates = MINI4AI_RX.duplicates + 1;
            end
            MINI4AI_RX.packets{idx} = p;
            MINI4AI_RX.lastSeq = max(MINI4AI_RX.lastSeq,seq);
        end
    end
end

function mini4aiCleanup(c)
    try, c.DataAvailableFcn = []; catch, end
    try, unsubscribe(c); catch, end
end
function v = leU16(p,i)
    v = double(p(i)) + 256*double(p(i+1));
end
function v = leU32(p,i)
    v = uint32(p(i)) + bitshift(uint32(p(i+1)),8) + ...
        bitshift(uint32(p(i+2)),16) + bitshift(uint32(p(i+3)),24);
end
function v = leI16(p,i)
    u = uint16(p(i)) + bitshift(uint16(p(i+1)),8);
    v = typecast(u,'int16');
end
