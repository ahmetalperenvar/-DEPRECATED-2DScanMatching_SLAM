function displayMap(Map,c) 

plot3(Map.grid(:,1),Map.grid(:,2),Map.grid(:,3), ['.' c], 'MarkerSize', 6);

end