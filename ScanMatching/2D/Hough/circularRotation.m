function b = circularRotation(a, cells)
	cells=round(cells);
	if cells>0
		b=[a(end-cells:end) a(1:end-cells-1)];
	else
		b=a;
	end
