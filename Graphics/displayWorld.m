function displayWorld(W)


     [Xmesh Ymesh] = meshgrid(W.df:W.df:size(W.Surface,1)*W.df, W.df:W.df:size(W.Surface,2)*W.df);

    %TextureImage=imread('mosaic.jpg');
    %Resize texture image at size of surface and set it to the correct
    %orientation for matching terrain bumps
    %TextureImage=imresize(TextureImage,size(Surface)); 
    %TextureImage=flipdim(W.TextureImage,1);

    % [Xmesh Ymesh] = meshgrid(min(profile(:,1)):0.1:max(profile(:,1)),min(profile(:,2)):0.1:max(profile(:,2)));
    % Z = griddata(profile(:,1),profile(:,2),profile(:,3),Xmesh,Ymesh);
    %TextureImage=imresize(TextureImage,size(Z)); 
    %TextureImage=flipdim(TextureImage,1);


     
     
    h = surface(Ymesh,Xmesh,W.Surface');
    shading flat
    
    %set(h,'CData',TextureImage,'FaceColor','texturemap');
end