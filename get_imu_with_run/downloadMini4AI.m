%% Mini4AI IMU log downloader for MATLAB R2025b
% Requires MG24 firmware with protocol version 2.
% ログは1サンプルずつ要求するlegacy方式（CMD_LOG_GET = 2）で取得する。
clear;

global MINI4AI_RX
MINI4AI_RX = struct('packets',{{}},'done',false,'lastSeq',-1,'duplicates',0);

% デバイス名「Mini4AI-IMU」を検索して接続
fprintf("Connecting...\n");
b = ble("Mini4AI-IMU");
chars = getMini4aiCharacteristics(b);

checkDeviceStatus(chars.status);
meta = requestMetadata(chars.cmd, chars.bulk);

data = downloadImuLog(chars.cmd, chars.bulk, meta.imuCount);

T = imuDataToTable(data);
filename = saveImuTable(T);
fprintf("Saved: %s\n",filename);

plotImuTable(T);

%% ============================================================
%  BLE connection
%  ============================================================
% 通信用の窓口オブジェクト取得
function chars = getMini4aiCharacteristics(b)
    SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0";
    CMD_UUID     = "12345678-1234-5678-1234-56789abcdef4";
    STATUS_UUID  = "12345678-1234-5678-1234-56789abcdef5";
    BULK_UUID    = "12345678-1234-5678-1234-56789abcdefc";

    chars.cmd    = characteristic(b,SERVICE_UUID,CMD_UUID);
    chars.status = characteristic(b,SERVICE_UUID,STATUS_UUID);
    chars.bulk   = characteristic(b,SERVICE_UUID,BULK_UUID);
end

% ロギング中・エラー状態ならダウンロードできないので中断
function checkDeviceStatus(statusChar)
    STATUS_ERROR   = 0;
    STATUS_LOGGING = 1;

    status = double(read(statusChar));
    fprintf("Status = %d\n",status);
    if status == STATUS_LOGGING
        error("MG24 is still logging. Turn it over and wait for DUMP_READY first.");
    elseif status == STATUS_ERROR
        error("MG24 reported ERROR.");
    end
end

% CMD_LOG_REQUEST(=1)でメタデータ（16byte）を取得して解析
function meta = requestMetadata(cmdChar, bulkChar)
    CMD_LOG_REQUEST = 1;

    write(cmdChar,uint8(CMD_LOG_REQUEST),"uint8");
    pause(0.25);
    p = uint8(read(bulkChar));
    if numel(p) < 16 || leU16(p,1) ~= hex2dec('FFFE')
        error("Metadata packet was not received.");
    end

    meta.imuCount        = double(leU16(p,4));   % IMUデータのサイズ（4byte目）
    meta.overflowCount   = double(leU16(p,6));   % オーバーフロー回数（6byte目）
    meta.protocolVersion = double(p(10));        % プロトコルバージョン（10byte目）
    meta.sampleHz        = double(leU32(p,13));  % サンプル周期（13byte目）

    fprintf("Samples=%d, Overflow=%d, Rate=%d Hz, Protocol=%d\n", ...
        meta.imuCount,meta.overflowCount,meta.sampleHz,meta.protocolVersion);
    if meta.protocolVersion < 2
        error("MG24 is not running the streaming firmware (protocol version 2).");
    end
end

%% ============================================================
%  Download
%  ============================================================
% 全サンプルを1つずつ要求して受信
% data: [timeUs gx gy gz ax ay az] の imuCount x 7 行列
function data = downloadImuLog(cmdChar, bulkChar, imuCount)
    PROGRESS_INTERVAL = 50;

    fprintf("Downloading logs (legacy mode)...\n");
    data = zeros(imuCount,7);
    t0 = tic;
    for k = 1:imuCount
        data(k,:) = requestSample(cmdChar, bulkChar, k - 1);
        if mod(k, PROGRESS_INTERVAL) == 0
            fprintf("Downloaded %d / %d\n", k, imuCount);
        end
    end
    fprintf("Completed in %.1f s\n", toc(t0));
end

% CMD_LOG_GET(=2)で seq 番目のサンプルを要求（seqは16bitリトルエンディアン）
function row = requestSample(cmdChar, bulkChar, seq)
    CMD_LOG_GET = 2;

    cmdPacket = uint8([CMD_LOG_GET, mod(seq, 256), floor(seq / 256)]);
    write(cmdChar, cmdPacket, "uint8");

    p = uint8(read(bulkChar));
    if numel(p) < 19
        error("Failed to read packet at sequence %d", seq);
    end
    row = parseSamplePacket(p);
end

% サンプルパケット（19byte）を解析
% ジャイロ：dps x 10、加速度：g x 1000 で送られてくる
function row = parseSamplePacket(p)
    timeUs = double(leU32(p,4));
    gyro   = double([leI16(p,8),  leI16(p,10), leI16(p,12)]) / 10;
    accel  = double([leI16(p,14), leI16(p,16), leI16(p,18)]) / 1000;
    row = [timeUs, gyro, accel];
end

%% ============================================================
%  Output
%  ============================================================
function T = imuDataToTable(data)
    T = table(data(:,1)/1e6,data(:,2),data(:,3),data(:,4),data(:,5),data(:,6),data(:,7), ...
        'VariableNames',{'time_s','gx_dps','gy_dps','gz_dps','ax_g','ay_g','az_g'});
end

% 受信したIMUデータをcsvファイルとして出力
function filename = saveImuTable(T)
    filename = "Mini4AI_" + string(datetime('now','Format','yyyyMMdd_HHmmss')) + ".csv";
    writetable(T,filename);
end

% 受信したIMUデータ可視化
function plotImuTable(T)
    figure;
    subplot(2,1,1); plot(T.time_s,[T.gx_dps T.gy_dps T.gz_dps]);
    grid on; xlabel("Time [s]"); ylabel("Gyro [dps]"); legend("gx","gy","gz");
    subplot(2,1,2); plot(T.time_s,[T.ax_g T.ay_g T.az_g]);
    grid on; xlabel("Time [s]"); ylabel("Accel [g]"); legend("ax","ay","az");
end

%% ============================================================
%  Streaming receive (CMD_STREAM_START = 6, currently unused)
%  ============================================================
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

%% ============================================================
%  Little-endian helpers
%  ============================================================
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
