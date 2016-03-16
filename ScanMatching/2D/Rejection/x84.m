function [ assp ixs ] = x84( assp )
%X84 This function post process a struct ASSP rejecting all the
%associations that do not match the Median Absolute Deviation rule over the
%residuals.

if size(assp.ref,1) < 5
    ixs=[];
    return
end

residuals = (assp.new - assp.ref).^2;
residuals = sqrt(residuals(:,1) + residuals(:,2) );


madass = mad(residuals);
K = 4;
rej_val = (K)*madass;

ixs = find(residuals > rej_val);

assp.ref(ixs,:) = [];
assp.new(ixs,:) = [];

end

