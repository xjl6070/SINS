classdef Serial_IMU_Data < handle
    properties
        s
        acc
        omega
        TimeStamp
        temperature
    end
    
    methods
        function obj = Serial_IMU_Data
            obj.s = serial("COM3","BaudRate",921600);
             obj.s.InputBufferSize = 8e6;
            fopen(obj.s);
        end
        function Read_IMU_Data(obj)
            persistent rcvAllBytes
            DataIsReceived = false;
            while 1
                aviBytesCount = obj.s.BytesAvailable;
                BytesCount = length(rcvAllBytes);
                if aviBytesCount > 0
                    rcvAllBytes(1 : BytesCount + aviBytesCount) = [rcvAllBytes , fread(obj.s,aviBytesCount,'uint8')'];
                    [~ , FrameHeadPos] = findsubmat(rcvAllBytes,[0xFF , 0x5A]);
                    if ~isempty(FrameHeadPos)
                        endPos = FrameHeadPos(end) +41 - 1;
                        if (BytesCount + aviBytesCount) >= (endPos)
                            if rcvAllBytes(endPos) == 0x33
                                NewFrame = rcvAllBytes(FrameHeadPos(end) : endPos);
                                obj.TimeStamp = typecast(uint8(NewFrame(7 : 10)), 'uint32');
                                obj.acc.x = double(typecast(uint8(NewFrame(13 : 16)), 'int32')) / 1048576;
                                obj.acc.y = double(typecast(uint8(NewFrame(17 : 20)), 'int32')) / 1048576;
                                obj.acc.z = double(typecast(uint8(NewFrame(21 : 24)), 'int32')) / 1048576;
                                obj.omega.x = double(typecast(uint8(NewFrame(25 : 28)), 'int32')) / 67108864;
                                obj.omega.y = double(typecast(uint8(NewFrame(29 : 32)), 'int32')) / 67108864;
                                obj.omega.z = double(typecast(uint8(NewFrame(33 : 36)), 'int32')) / 67108864;
                                obj.temperature = typecast(uint8(NewFrame(37 : 38)), 'int16')/256;
                                DataIsReceived = true;                                
                            end
                        else
                            continue;
                        end
                        rcvAllBytes = rcvAllBytes(endPos+1: end);
                    end
                end
                if DataIsReceived
                    break
                end
            end
        end
    end
end