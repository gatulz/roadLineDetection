function out = modx(inp,rad)
    max = 180;
    if (rad)
%         max = degtorad(max);
      inp = radtodeg(inp);
    end
    if (inp>max)
        out = inp - 360
    elseif (inp<-max)
        out = inp + 360;
    else
        out = inp;
    end
    if (rad)
        out = degtorad(out);
        
end