clear
clc

global IMUDATA DONEFLAG

IMUDATA = {};
DONEFLAG = false;

%% UUID

SERVICE_UUID = ...
"12345678-1234-5678-1234-56789abcdef0";

CMD_UUID = ...
"12345678-1234-5678-1234-56789abcdef4";

STATUS_UUID = ...
"12345678-1234-5678-1234-56789abcdef5";

BULK_UUID = ...
"12345678-1234-5678-1234-56789abcdefc";

%% Connect

disp("Connecting...")

b = ble("Mini4AI-IMU");

cmdChar = characteristic( ...
    b,...
    SERVICE_UUID,...
    CMD_UUID);

statusChar = characteristic( ...
    b,...
    SERVICE_UUID,...
    STATUS_UUID);

bulkChar = characteristic( ...
    b,...
    SERVICE_UUID,...
    BULK_UUID);

%% Metadata

write(cmdChar,uint8(1),"uint8");

pause(0.2)

meta = uint8(read(bulkChar));

imuCount = double(typecast(uint8(meta(4:5)),'uint16'));

fprintf("imuCount = %d\n",imuCount);

%% Subscribe

subscribe(bulkChar);

bulkChar.DataAvailableFcn = @rxCallback;

%% Start Stream

disp("Start streaming...")

CMD_STREAM_START = uint8(6);

write(cmdChar,...
    uint8([CMD_STREAM_START 0 0]),...
    "uint8");

%% Wait

tic

while ~DONEFLAG

    pause(0.05);

    if toc > 120

        warning("timeout")

        break
    end

end

elapsed = toc;

fprintf("Transfer Time = %.1f sec\n",elapsed);

%% Stop

bulkChar.DataAvailableFcn = [];

unsubscribe(bulkChar);

%% Decode

N = length(IMUDATA);

fprintf("Received packets = %d\n",N);

timeUs = zeros(N,1);

gx = zeros(N,1);
gy = zeros(N,1);
gz = zeros(N,1);

ax = zeros(N,1);
ay = zeros(N,1);
az = zeros(N,1);

for k = 1:N

    pkt = IMUDATA{k};

    timeUs(k) = ...
        double(typecast(uint8(pkt(4:7)),'uint32'));

    gx(k) = ...
        double(typecast(uint8(pkt(8:9)),'int16'))/10;

    gy(k) = ...
        double(typecast(uint8(pkt(10:11)),'int16'))/10;

    gz(k) = ...
        double(typecast(uint8(pkt(12:13)),'int16'))/10;

    ax(k) = ...
        double(typecast(uint8(pkt(14:15)),'int16'))/1000;

    ay(k) = ...
        double(typecast(uint8(pkt(16:17)),'int16'))/1000;

    az(k) = ...
        double(typecast(uint8(pkt(18:19)),'int16'))/1000;

end

%% Table

T = table( ...
    timeUs/1e6,...
    gx,gy,gz,...
    ax,ay,az,...
    'VariableNames',...
    {'time_s',...
    'gx_dps','gy_dps','gz_dps',...
    'ax_g','ay_g','az_g'});

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

legend("gx","gy","gz")

grid on

subplot(2,1,2)

plot(T.time_s,T.ax_g)
hold on
plot(T.time_s,T.ay_g)
plot(T.time_s,T.az_g)

legend("ax","ay","az")

grid on