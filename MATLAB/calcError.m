function errorTheta = calcError(target, actual, rad)
    if (rad)
        target = radtodeg(target);
        actual = radtodeg(actual);
    end
    if (target < actual)
        calc1 = target - actual; %90
        calc2 = 360 - actual + target; %270
        if (abs(calc1) <= calc2)
            errorTheta = calc1;
        else
            errorTheta = calc2;
        end
    elseif (target > actual)
        errorTheta = target - actual;
%         calc2 = 360 - target + actual;
%         if (calc1 <= calc2)
%             errorTheta = calc1;
%         else
%             errorTheta = calc2;
%         end
    elseif (target == actual)
        errorTheta = 0;
    end
%     errorTheta = errorTheta - 180;
    if (rad)
        errorTheta = degtorad(errorTheta);
    end
end

