 clear;
types = ["Vehicle" "Character" "OtherMovingObject" "TrafficSignal" "3DModels" "Signs" "RoadMarkings"];

uReceiver = udpport("LocalPort", 3030);

while true
    if uReceiver.NumBytesAvailable > 0
        data = read(uReceiver, uReceiver.NumBytesAvailable);
        dispComm(data, types);
    end
end

function readAcknowledgement(u, ~)
data = readline(u);

disp(data);
end