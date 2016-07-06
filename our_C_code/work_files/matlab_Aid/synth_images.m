% plot moving square 
W = 
dx = 0.1 ; % 0.01  % dx per frame
r = 1 ; 
x = [0: dx : 3]; 
V = VideoWriter('square.avi'); 
open(V); 
choose_shape = 3 ; % 1- rect, 2-triangle , 3-circle
% choose_fg_color = ..  % influencing ? 
% choose_bg_color = ..  % influencing ? 
 ver = [x(i) 0; x(i)+1 0; x(i)+1 1; x(i)+0 1]; axis equal 
for i=1:length(x) 
    if (choose_shape==1) , ver = [x(i) 0; x(i)+1 0; x(i)+1 1; x(i)+0 1]; end
     if (choose_shape==2) , ver = [ x(i) 0.0; x(i) 1; x(i)+1 0.5];end
     
   if (choose_shape==3) 
       r = 1;
       theta = 0:0.01:2*3.14;
       ver = [ x(i)+r*cos(theta); 1.1+ r*sin(theta)]';
   end
    clf('reset');    
    h = patch(ver(:,1),ver(:,2),'r');  
    axis([-1 5 -1 5]); 
%     axis equal 
    writeVideo(V,getframe()) 
end 
close(V) 
