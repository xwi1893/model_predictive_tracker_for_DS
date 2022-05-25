function dispComm(data, types)
%DISPCOMM Summary of this function goes here
    nData = length(data);
    lenObj = 60;
    nObject = int32((nData - 12)/lenObj);
    time = typecast(uint8(data(1:8)), 'double');
    fprintf("Current time: %0.2f s \n", time);
    if nObject ~= typecast(uint8(data(9:12)), 'int32')
        error('The number of objects is incorrect!');
    end

    len = [4, 8, 8, 8, 8, 8, 8, 4, 4];
    index = [0 cumsum(len)];
    names = ["ID" "X" "Y" "Z" "Roll" "Pitch" "Yaw" "Sensor ID" "Type"];
    for i = 1:nObject
        frameData = data(12+lenObj*(i-1)+1 : 12+lenObj*i);
        for j = 1:numel(len)
            if len(j) == 4
                datatype = 'int32';
            else
                datatype = 'double';
            end
            if j < numel(len)
                dispRow(names(j), frameData(index(j)+1:index(j+1)), datatype);
            else
                dispRow(names(j), frameData(index(j)+1:index(j+1)), datatype, types);
            end
        end
    end
    fprintf("-------------------------------- \n");
end

function dispRow(varargin)
    name = varargin{1};
    data = varargin{2};
    datatype = varargin{3};

    data = typecast(uint8(data), datatype);
    if strcmp(datatype, 'int32')
        if nargin == 4
            types = varargin{4};
            type = types(data);
            fprintf([name+": %s \n"], type);
        else
            fprintf([name+ ": %d \n"], data);
        end
    elseif strcmp(datatype, 'double')
        fprintf([name+": %0.2f \n"], data)
    end
end
