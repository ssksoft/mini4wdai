%% Mini4AI IMU stream downloader for MATLAB R2025b
% Requires MG24 firmware with CMD_STREAM_START = 6.
clear;

global MINI4AI_RX
MINI4AI_RX = struct('packets',{{}},'done',false,'lastSeq',-1,'duplicates',0);

% 接続先情報
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0";
CMD_UUID     = "12345678-1234-5678-1234-56789abcdef4";
STATUS_UUID  = "12345678-1234-5678-1234-56789abcdef5";
BULK_UUID    = "12345678-1234-5678-1234-56789abcdefc";

% デバイス名「Mini4AI-IMU」を検索して接続
fprintf("Connecting...\n");
b = ble("Mini4AI-IMU");

% 通信用の窓口オブジェクト取得
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

% メタデータのチェック
write(cmdChar,uint8(1),"uint8");
pause(0.25);
meta = uint8(read(bulkChar));
if numel(meta) < 16 || leU16(meta,1) ~= hex2dec('FFFE')
    error("Metadata packet was not received.");
end

// IMUデータのサイズ（metaデータの4byte目）を取得
imuCount = double(leU16(meta,4));
// IMUデータのオーバーフロー回数（metaデータの6byte目）を取得
overflowCount = double(leU16(meta,6));
// IMUデータのサンプル周期（metaデータの13byte目）を取得
sampleHz = double(leU32(meta,13));
// プロトコルバージョン（metaデータの10byte目）を取得
protocolVersion = double(meta(10));
fprintf("Samples=%d, Overflow=%d, Rate=%d Hz, Protocol=%d\n", ...
    imuCount,overflowCount,sampleHz,protocolVersion);
if protocolVersion < 2
    error("MG24 is not running the streaming firmware (protocol version 2).");
end


fprintf("Downloading logs (legacy mode)...\n");
timeUs = zeros(imuCount,1);
gx = zeros(imuCount,1); gy = zeros(imuCount,1); gz = zeros(imuCount,1);
ax = zeros(imuCount,1); ay = zeros(imuCount,1); az = zeros(imuCount,1);

% BLEでデータ取得
t0 = tic;
for k = 1:imuCount
    % k-1番目のデータを取得
    seq = k - 1;
    % CMD_LOG_SGET(=2)コマンドを実行,要求したいデータの番号の16bitのうち下位8bitを取得,要求したいデータの番号の16bitのうち上位8bitを取得
    cmdPacket = [2, uint8(mod(seq, 256)), uint8(floor(seq / 256))];

    % BLEコマンド送信
    write(cmdChar, uint8(cmdPacket), "uint8");
    
    % 受信データをpに格納（1パケット19byte）
    p = uint8(read(bulkChar));
    if numel(p) < 19
        error("Failed to read packet at sequence %d", seq);
    end
    
    timeUs(k) = double(leU32(p,4));
    % 精度：0.1
    gx(k) = double(leI16(p,8))/10;
    gy(k) = double(leI16(p,10))/10;
    gz(k) = double(leI16(p,12))/10;
    % 精度：0.001
    ax(k) = double(leI16(p,14))/1000;
    ay(k) = double(leI16(p,16))/1000;
    az(k) = double(leI16(p,18))/1000;
    
    % カウントが50の倍数になったときにステータス表示
    if mod(k, 50) == 0
        fprintf("Downloaded %d / %d\n", k, imuCount);
    end
end
elapsed = toc(t0);
fprintf("Completed in %.1f s\n", elapsed);

% 受信したIMUデータをcsvファイルとして出力
T = table(timeUs/1e6,gx,gy,gz,ax,ay,az, ...
    'VariableNames',{'time_s','gx_dps','gy_dps','gz_dps','ax_g','ay_g','az_g'});
filename = "Mini4AI_" + string(datetime('now','Format','yyyyMMdd_HHmmss')) + ".csv";
writetable(T,filename);
fprintf("Saved: %s\n",filename);

% 受信したIMUデータ可視化
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
