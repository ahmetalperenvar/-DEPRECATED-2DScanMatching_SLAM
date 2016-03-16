function [ o ] = displayAssociations( assp, hp )
%DISPLAYASSOCIATIONS Summary of this function goes here
%   Detailed explanation goes here
o=1;
if  ~isempty(assp)
    plot_assoc = [];
    for i = 1:size(assp.new,1)
        plot_assoc = [plot_assoc; [assp.new(i,1:2); assp.ref(i,1:2); NaN NaN ] ];
    end
    
    set(hp,'XData',plot_assoc(:,1),'YData',plot_assoc(:,2));
end

end

