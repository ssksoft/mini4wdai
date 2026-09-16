%% downloadMini4AI.m
% XIAO MG24 IMU log downloader for MATLAB R2025b
% Compatible with MG24 firmware using CMD_STREAM_START = 6.
% The callback is included at the end of this same file.

clear;
clc;

%% Shared receive state
global MINI4AI_RX
MINI4AI_RX = struct( ...
    'packets', {{}}, ...
    'received', 0, ...
    'done', false, ...
    'terminatorCount', 0, ...
    'callbackErrors', 0);

%% BLE UUIDs
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0";
CMD_UUID     = "12345678-1234-5678-1234-56789abcdef4";
STATUS_UUID  = "12345678-1234-5678-1234-56789abcdef5";
BULK_UUID    = "12345678-1234-5678-1234-56789abcdefc";

CMD_LOG_REQUEST  = uint8(1);
CMD_STREAM_START = uint8(6);

%% Connect
fprintf("Connecting to Mini4AI-IMU...\n");
b = ble("Mini4AI-IMU");

cmdChar = characteristic(b, SERVICE_UUID, CMD_UUID);
statusChar = characteristic(b, SERVICE_UUID, STATUS_UUID);
bulkChar = characteristic(b, SERVICE_UUID, BULK_UUID);

status = double(read(statusChar));
fprintf("Status = %d\n", status);

if status == 0
    error("MG24 reported STATUS_ERROR.");
elseif status == 1
    error("MG24 is still logging. Turn the car over and wait for logging to stop.");
end

%% Request metadata
% Do not use read(...,'oldest') here because notifications have not yet
% been subscribed at this point.
fprintf("Requesting metadata...\n");
write(cmdChar, CMD_LOG_REQUEST, "uint8");
pause(0.3);
meta = uint8(read(bulkChar));

if numel(meta) < 16
    error("Metadata packet is too short: %d bytes.", numel(meta));
end

metaMarker = leU16(meta, 1);
if metaMarker ~= hex2dec('FFFE')
    error("Metadata marker 0xFFFE was not received. Received marker: 0x%04X", metaMarker);
end

imuCount = double(leU16(meta, 4));
overflowCount = double(leU16(meta, 6));
protocolVersion = double(meta(10));
sampleHz = double(leU32(meta, 13));

fprintf("Samples          = %d\n", imuCount);
fprintf("Overflow         = %d\n", overflowCount);
fprintf("Sample rate      = %d Hz\n", sampleHz);
fprintf("Protocol version = %d\n", protocolVersion);

if imuCount == 0
    error("No IMU samples are stored on MG24.");
end
if protocolVersion < 2
    error("MG24 firmware does not support CMD_STREAM_START = 6.");
end

%% Prepare receive buffer
% Store packets by their sequence number, not arrival order.
MINI4AI_RX.packets = cell(imuCount, 1);
MINI4AI_RX.received = 0;
MINI4AI_RX.done = false;
MINI4AI_RX.terminatorCount = 0;
MINI4AI_RX.callbackErrors = 0;

%% Subscribe and start streaming
subscribe(bulkChar);
bulkChar.DataAvailableFcn = @mini4aiRxCallback;
cleanupObj = onCleanup(@()cleanupBle(bulkChar)); %#ok<NASGU>

fprintf("Downloading IMU log...\n");
tStart = tic;
write(cmdChar, uint8([CMD_STREAM_START 0 0]), "uint8");

%% Wait for terminator
TIMEOUT_SEC = 180;
lastDisplayed = -1;

% while ~MINI4AI_RX.done && toc(tStart) < TIMEOUT_SEC
while ~MINI4AI_RX.done && MINI4AI_RX.received < imuCount
    pause(0.02);
    drawnow limitrate;

    % Display progress in the main thread.
    progressBlock = floor(MINI4AI_RX.received / 100);
    if progressBlock > lastDisplayed
        fprintf("Packet %d / %d\n", MINI4AI_RX.received, imuCount);
        lastDisplayed = progressBlock;
    end
end

transferSec = toc(tStart);

% Disable callback before processing shared data.
bulkChar.DataAvailableFcn = [];
unsubscribe(bulkChar);

fprintf("Received packets = %d / %d\n", MINI4AI_RX.received, imuCount);
fprintf("Transfer time    = %.1f s\n", transferSec);
fprintf("Callback errors  = %d\n", MINI4AI_RX.callbackErrors);

if ~MINI4AI_RX.done
    error("Timed out before receiving the terminator packet.");
end

%% Check missing packets
receivedMask = ~cellfun('isempty', MINI4AI_RX.packets);
missingSeq = find(~receivedMask) - 1;

if ~isempty(missingSeq)
    error("%d packets are missing. First missing sequence is %d. Reduce STREAM_BURST on MG24.", ...
        numel(missingSeq), missingSeq(1));
end

%% Decode all stored packets
N = imuCount;
timeUs = zeros(N, 1);
gx = zeros(N, 1);
gy = zeros(N, 1);
gz = zeros(N, 1);
ax = zeros(N, 1);
ay = zeros(N, 1);
az = zeros(N, 1);

for k = 1:N
    pkt = MINI4AI_RX.packets{k};

    timeUs(k) = double(leU32(pkt, 4));
    gx(k) = double(leI16(pkt, 8))  / 10.0;
    gy(k) = double(leI16(pkt, 10)) / 10.0;
    gz(k) = double(leI16(pkt, 12)) / 10.0;
    ax(k) = double(leI16(pkt, 14)) / 1000.0;
    ay(k) = double(leI16(pkt, 16)) / 1000.0;
    az(k) = double(leI16(pkt, 18)) / 1000.0;
end

%% Create workspace table
T = table( ...
    timeUs / 1e6, ...
    gx, gy, gz, ...
    ax, ay, az, ...
    'VariableNames', { ...
    'time_s', ...
    'gx_dps', 'gy_dps', 'gz_dps', ...
    'ax_g', 'ay_g', 'az_g'});

fprintf("Decoded rows = %d\n", height(T));
fprintf("Size of ax   = %s\n", mat2str(size(ax)));

%% Save CSV
stamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
filename = "Mini4AI_" + stamp + ".csv";
writetable(T, filename);
fprintf("Saved: %s\n", filename);

%% Plot
figure('Name', 'Mini4AI IMU Log');

subplot(2, 1, 1);
plot(T.time_s, T.gx_dps, ...
     T.time_s, T.gy_dps, ...
     T.time_s, T.gz_dps);
grid on;
xlabel("Time [s]");
ylabel("Gyro [dps]");
legend("gx", "gy", "gz", 'Location', 'best');

subplot(2, 1, 2);
plot(T.time_s, T.ax_g, ...
     T.time_s, T.ay_g, ...
     T.time_s, T.az_g);
grid on;
xlabel("Time [s]");
ylabel("Acceleration [g]");
legend("ax", "ay", "az", 'Location', 'best');

fprintf("Done.\n");

%% Local callback function
function mini4aiRxCallback(src, ~)
    global MINI4AI_RX

    try
        % DataAvailableFcn has been called for a notification. Read one
        % queued notification. NumBytesAvailable is not used by BLE objects.
        pkt = uint8(read(src, 'oldest'));

        if numel(pkt) < 2
            return;
        end

        marker = leU16(pkt, 1);

        % Terminator packet: first two bytes are 0xFFFF.
        if marker == hex2dec('FFFF')
            MINI4AI_RX.terminatorCount = MINI4AI_RX.terminatorCount + 1;
            MINI4AI_RX.done = true;
            return;
        end

        % Ignore metadata and malformed packets.
        if numel(pkt) ~= 19 || pkt(3) ~= 1
            return;
        end

        sequence = double(marker);
        index = sequence + 1;

        if index < 1 || index > numel(MINI4AI_RX.packets)
            return;
        end

        % Count only the first arrival of each sequence.
        if isempty(MINI4AI_RX.packets{index})
            MINI4AI_RX.packets{index} = pkt;
            MINI4AI_RX.received = MINI4AI_RX.received + 1;
        end

    catch ME
        MINI4AI_RX.callbackErrors = MINI4AI_RX.callbackErrors + 1;
        fprintf("Callback error: %s\n", ME.message);
    end
end

%% Local utility functions
function cleanupBle(c)
    try
        c.DataAvailableFcn = [];
    catch
    end
    try
        unsubscribe(c);
    catch
    end
end

function value = leU16(p, index)
    value = double(p(index)) + 256 * double(p(index + 1));
end

function value = leU32(p, index)
    value = uint32(p(index)) ...
        + bitshift(uint32(p(index + 1)), 8) ...
        + bitshift(uint32(p(index + 2)), 16) ...
        + bitshift(uint32(p(index + 3)), 24);
end

function value = leI16(p, index)
    raw = uint16(p(index)) + bitshift(uint16(p(index + 1)), 8);
    value = typecast(raw, 'int16');
end
