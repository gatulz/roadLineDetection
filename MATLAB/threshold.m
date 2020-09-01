function out = threshold(inp, max, rad)
    if (rad)
        max = degtorad(max);
    end
    if (inp>max)
        out = max;
    elseif (inp<-max)
        out = -max;
    else
        out = inp;
    end
        
end