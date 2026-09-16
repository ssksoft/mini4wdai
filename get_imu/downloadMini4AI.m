%% ==========================================================
% Mini4AI IMU Logger Downloader
% XIAO MG24 + LSM6DS3
% ===========================================================

clear
clc

%% UUID

SERVICE_UUID = ...
    "12345678-1234-5678-1234-56789abcdef0";

CMD_UUID = ...
    "12345678-1234-5678-1234-56789abcdef4";

STATUS_UUID = ...
    "12345678-1234-5678-1234-56789abcdef5";

BULK_UUID = ...
    "12345678-1234-5678-1234-56789abcdefc";

%% Commands

CMD_LOG_REQUEST = uint8(1);
CMD_LOG_GET     = uint8(2);

%% BLE Scan

disp("Searching BLE devices...")
disp(blelist)

%% Connect

disp("Connecting...")

b = ble("Mini4AI-IMU");

%% Characteristics

cmdChar = characteristic( ...
    b, ...
    SERVICE_UUID, ...
    CMD_UUID);

statusChar = characteristic( ...
    b, ...
    SERVICE_UUID, ...
    STATUS_UUID);

bulkChar = characteristic( ...
    b, ...
    SERVICE_UUID, ...
    BULK_UUID);

%% Read Status

status = read(statusChar);

fprintf("Status = %d\n",status);

switch status
    case 0
        error("Device ERROR");

    case 1
        warning("Still LOGGING");

    case 2
        disp("DUMP READY");

    case 3
        disp("DUMPING");
end

%% Request Metadata

disp("Request metadata...")

write(cmdChar,CMD_LOG_REQUEST,"uint8");

pause(0.2)

meta = read(bulkChar);

if length(meta) < 16
    error("Invalid metadata packet")
end

hdr = typecast(uint8(meta(1:2)),'uint16');

if hdr ~= hex2dec('FFFE')
    error("Metadata header not found")
end

imuCount = double(typecast(uint8(meta(4:5)),'uint16'));

overflowCount = ...
    double(typecast(uint8(meta(6:7)),'uint16'));

sampleHz = ...
    double(typecast(uint8(meta(13:16)),'uint32'));

fprintf("\n");
fprintf("Samples      : %d\n",imuCount);
fprintf("Overflow     : %d\n",overflowCount);
fprintf("Sample Rate  : %d Hz\n",sampleHz);
fprintf("\n");

%% Allocate

timeUs = zeros(imuCount,1);

gx = zeros(imuCount,1);
gy = zeros(imuCount,1);
gz = zeros(imuCount,1);

ax = zeros(imuCount,1);
ay = zeros(imuCount,1);
az = zeros(imuCount,1);

%% Download

disp("Downloading log ...")

h = waitbar(0,"Downloading...");

for seq = 0:(imuCount-1)

    %% Build command

    cmd = zeros(1,3,'uint8');

    cmd(1) = CMD_LOG_GET;
    cmd(2) = bitand(seq,255);
    cmd(3) = bitshift(seq,-8);

    %% Send request

    write(cmdChar,cmd,"uint8");

    pause(0.005)

    pkt = read(bulkChar);

    if length(pkt) < 19
        warning("Invalid packet %d",seq)
        continue
    end

    idx = seq + 1;

    %% Decode

    timeUs(idx) = ...
        double(typecast(uint8(pkt(4:7)),'uint32'));

    gx(idx) = ...
        double(typecast(uint8(pkt(8:9)),'int16')) ...
        / 10.0;

    gy(idx) = ...
        double(typecast(uint8(pkt(10:11)),'int16')) ...
        / 10.0;

    gz(idx) = ...
        double(typecast(uint8(pkt(12:13)),'int16')) ...
        / 10.0;

    ax(idx) = ...
        double(typecast(uint8(pkt(14:15)),'int16')) ...
        / 1000.0;

    ay(idx) = ...
        double(typecast(uint8(pkt(16:17)),'int16')) ...
        / 1000.0;

    az(idx) = ...
        double(typecast(uint8(pkt(18:19)),'int16')) ...
        / 1000.0;

    if mod(seq,20)==0
        waitbar(seq/imuCount,h)
    end

end

close(h)

%% Table

T = table;

T.time_s = timeUs / 1e6;

T.gx_dps = gx;
T.gy_dps = gy;
T.gz_dps = gz;

T.ax_g = ax;
T.ay_g = ay;
T.az_g = az;

%% Save

filename = sprintf( ...
    "Mini4AI_%s.csv", ...
    datestr(now,'yyyymmdd_HHMMSS'));

writetable(T,filename)

fprintf("Saved : %s\n",filename);

%% Plot

figure

subplot(2,1,1)

plot(T.time_s,T.gx_dps)
hold on
plot(T.time_s,T.gy_dps)
plot(T.time_s,T.gz_dps)

grid on

xlabel("Time [s]")
ylabel("Gyro [dps]")

legend("gx","gy","gz")

subplot(2,1,2)

plot(T.time_s,T.ax_g)
hold on
plot(T.time_s,T.ay_g)
plot(T.time_s,T.az_g)

grid on

xlabel("Time [s]")
ylabel("Accel [g]")

legend("ax","ay","az")

disp("Done.")