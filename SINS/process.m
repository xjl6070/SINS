function process(k,N) 
dots = '....................';
bars = '||||||||||||||||||||';
rewind = '\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b';
persistent progress2_epoch progress2_mark
if isempty(progress2_epoch)||isempty(progress2_mark)
    progress2_epoch = 0;    progress2_mark = 0;
    fprintf(strcat('Processing: ',dots));
end
if (k - progress2_epoch) >= N/20
    progress2_mark = progress2_mark + 1;
    progress2_epoch = k;
    fprintf(strcat(rewind,bars(1:progress2_mark),dots(1:(20 - progress2_mark))));
end
if k == N
    fprintf(strcat(rewind,bars,'\n'));
end