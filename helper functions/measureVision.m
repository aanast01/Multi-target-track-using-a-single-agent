function measurement = measureVision(tgt, agentZ)

    pdFunValue = -0.0083*agentZ+1.083;

    if(agentZ <= 10)
        pd = 1;
    elseif(agentZ >= 100)
        pd = 0.25;
    else
        pd = pdFunValue;
    end
    
    sigma = 0.001 / pd;
    
    measurement = squeeze(tgt) + (sigma*agentZ).*randn(length(tgt),1);
    
end