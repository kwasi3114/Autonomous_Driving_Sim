function webotsCameraSFunction(block)
    setup(block);
end

function setup(block)
    block.NumInputPorts  = 0;
    block.NumOutputPorts = 1;
    block.SetPreCompOutputPortDimensions(1, [480, 640, 3]); % Image size
    block.SetPreCompOutputPortDatatype(1, 'uint8');

    block.RegBlockMethod('Outputs', @Outputs);
end

function Outputs(block)
    global webotsImage;
    if isempty(webotsImage)
        block.OutputPort(1).Data = zeros(480, 640, 3, 'uint8');
    else
        block.OutputPort(1).Data = webotsImage;
    end
end
