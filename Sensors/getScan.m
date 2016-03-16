%Retrieves a previous scan with index i from a map object
function S = getScan(Map,i)

S = [];

if i <= 0
    return
end

s = Map.prev.written(i);

S.data.localCart = Map.prev.gridlocal(i).points;
S.data.globalPolar = Map.prev.gridpolar(i).points;
S.data.localPolar = Map.prev.gridlocalpolar(i).points;
S.data.covm = Map.prev.gridcovm(i).points;

%S.data.globalCart = Map.grid( ( (i-1) *s) + 1 : (i*s)  ,:);


end
