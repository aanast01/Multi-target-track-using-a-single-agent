function measurement = measureRadar(tgt)
    sigma = 1;
    
    measurement = squeeze(tgt) + sigma.*randn(length(tgt),1);
    
end