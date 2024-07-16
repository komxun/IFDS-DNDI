function UpdateWM(obj)

    z_i = obj.itsCurrentPos(3);
    mapSpan = obj.DA.Param.mapSpan;
    % weatherMatMod = obj.DA.weatherMatMod;
    weatherMatMod = obj.DA.weatherMat;
    flatAlt = max(weatherMatMod(weatherMatMod <= z_i - obj.deltaH));
    weatherMatMod(weatherMatMod < z_i - obj.deltaH) = flatAlt;

    WMCell = cell(1,size(weatherMatMod,3));
    dwdxCell = cell(1,size(weatherMatMod,3));
    dwdyCell = cell(1,size(weatherMatMod,3));

    xspace = 1:mapSpan;
    yspace = 1:mapSpan;
    [xgrid, ygrid] = meshgrid(xspace, yspace);

    for j = 1:size(weatherMatMod,3)
        WMCell{j} = griddedInterpolant(weatherMatMod(:,:,j)');
        z_values = WMCell{j}(xgrid,ygrid);
        [grad_x, grad_y] = gradient(z_values, xspace, yspace);
        dwdxCell{j} = griddedInterpolant(grad_x');
        dwdyCell{j} = griddedInterpolant(grad_y');
    end

    % Update values
    disp("*" + obj.name + ": Updating terrain data . . . at t = " + num2str(obj.rt/100) + " s")
    obj.DA.weatherMatMod = weatherMatMod;
    obj.DA.WMCell = WMCell;
    obj.DA.dwdxCell = dwdxCell;
    obj.DA.dwdyCell = dwdyCell;

end