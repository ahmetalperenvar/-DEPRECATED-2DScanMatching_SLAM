function [theNDT]=build_NDT(S, cellSize, offsetX, offsetY)
  % Store parameters so that theNDT structure is self-contained.
  theNDT.theScan=S;
  theNDT.cellSize=cellSize;
  % Compute the limit coordinates of the scan
  theNDT.Xmin=min(S(1,:));
  theNDT.Ymin=min(S(2,:));
  theNDT.Xmax=max(S(1,:));
  theNDT.Ymax=max(S(2,:));
  theNDT.offsetX=offsetX;
  theNDT.offsetY=offsetY;
  % Compute the grid size
  theNDT.gridWidth=ceil((theNDT.Xmax-theNDT.Xmin)/cellSize);
  theNDT.gridHeight=ceil((theNDT.Ymax-theNDT.Ymin)/cellSize);
  % For each grid cell,
  for i=1:theNDT.gridWidth,
      for j=1:theNDT.gridHeight,
          % Look for the scan points lying inside.
          theNDT.theGrid(j,i).theScanIndexes=find(S(1,:)>=theNDT.Xmin+(i-1)*cellSize-offsetX & S(1,:)<=theNDT.Xmin+i*cellSize-offsetX & S(2,:)>=theNDT.Ymin+(j-1)*cellSize-offsetY & S(2,:)<=theNDT.Ymin+j*cellSize-offsetY);
          % Compute mean and covariance.
          if (size(theNDT.theGrid(j,i).theScanIndexes,2)>2),
            thePoints=[S(1,theNDT.theGrid(j,i).theScanIndexes);S(2,theNDT.theGrid(j,i).theScanIndexes)];
            theNDT.theGrid(j,i).theMean=mean(thePoints')';
            theNDT.theGrid(j,i).theCovariance=cov(thePoints')';
            
            theMinEigVal=min(eig(theNDT.theGrid(j,i).theCovariance));
            theMaxEigVal=max(eig(theNDT.theGrid(j,i).theCovariance));
            while (theMinEigVal/theMaxEigVal<0.001),
              theNDT.theGrid(j,i).theCovariance=theNDT.theGrid(j,i).theCovariance+eye(2)*0.0001;
              theMinEigVal=min(eig(theNDT.theGrid(j,i).theCovariance));
              theMaxEigVal=max(eig(theNDT.theGrid(j,i).theCovariance));
            end;                               
            theNDT.theGrid(j,i).theInvertedCovariance=inv(theNDT.theGrid(j,i).theCovariance);
            theNDT.theGrid(j,i).exists=1;              
          else
            theNDT.theGrid(j,i).theMean=-1;
            theNDT.theGrid(j,i).theCovariance=-1;
            theNDT.theGrid(j,i).theInvertedCovariance=-1;
            theNDT.theGrid(j,i).exists=0;
          end;          
      end;
  end;
return;