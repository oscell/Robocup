map = binaryOccupancyMap(11,9,100);
setOccupancy(map,[5 5], 1);

setOccupancy(map,[7 7], 1);

inflate(map,0.15);
show(map)