% plot moving square 
x = [0: 0.1 : 3]; 
V = VideoWriter('square.avi'); 
open(V); 
for i=1:length(x) 
%     ver = [x(i) 0; x(i)+1 0; x(i)+1 1];
    ver = [ x(i) 0.0; x(i) 1; x(i)+1 0.5];
%     ver = [x(i) 0; x(i)+1 0; x(i)+1 1; x(i)+0 1];
    clf('reset');   
    h = patch(ver(:,1),ver(:,2),'r'); 
    axis([-1 5 -1 5]); 
%     axis equal 
    writeVideo(V,getframe()) 
end 
close(V) 
