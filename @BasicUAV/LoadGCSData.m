function LoadGCSData(obj, topo, R, lat0, lon0, h0, WMCell, dwdxCell, dwdyCell)

    % z_i = obj.itsCurrentPos(3);
    disp("Loading Terrain Matrix from GCS...")
    
    % Terrain
    obj.topo = topo;
    obj.R = R;
    obj.lat0 = lat0;
    obj.lon0 = lon0;
    obj.h0 = h0;

    % Save into DA
    obj.DA.WMCell = WMCell;
    obj.DA.weatherMat = topo;
    % obj.DA.weatherMatMod = terrainMatMod;
    obj.DA.dwdxCell = dwdxCell;
    obj.DA.dwdyCell = dwdyCell;

end