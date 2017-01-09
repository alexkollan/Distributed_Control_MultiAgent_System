%ecg to bpm 


%count the domimamt peaks
beat_count = 0;
for k = 2 : length(sig) - 1
if(sig(k) > sig(k-1) & sig(k) > sig(k+1) & sig(k) > 1)
    %k
    %disp('Prominant peak Found!');
    beat_count = beat_count + 1;
end
end


%Divide by signal duration in minutes
beat_count
fs = 100;
N = length(sig);
dur_sec = N/fs;
dur_min = dur_sec/60;
BPM = beat_count/dur_min;
disp(BPM);