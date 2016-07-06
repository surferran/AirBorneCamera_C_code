L = 400;
D = 80;
speckleSize = L/D
img = zeros(L, L);
pad = L/2; % padding
for k=pad-D:D+pad
  for l=pad-D:D+pad
    if abs((pad-k)^2+(pad-l)^2) < D^2/4
      img(k,l) = exp(unifrnd(-pi,pi));
    end
  end
end
mfft = fft2(img);
img = mfft.*conj(mfft);
subimg = img(20:380, 20:380);
figure; imshow(subimg, []);
% %%
% np=1;
% data5 = img;
% npp=16*np;                % np gives original test image size
% 
%                             % npp is up_converted test image size
% 
% phase0= rand(npp,npp);
% 
% data0= exp(i*2*pi*phase0);   % random phasor creation
% 
% data5= data5.*data0;        % data5 is test image up_converted
% 
% image= zeros(np,np);        % image is for speckled image after down conversion
% 
% data1=data10;
% 
% M=200;                              % temporal diversity M=200
% 
% for m=1:M;
% 
% data1= circshift(data1,5,1);  % moving diffuser rotation
% 
% F= fft2(data5.*data1);
% 
% Fp= F(1 : np,1 : np);        % down_conversion: low pass filtering
% 
% f= abs(ifft2(Fp));
% 
% image= image+f.^2;         
% 
% end;
%%

Img = zeros(512,512);

ramp = 2^16*linspace(0,1,128);
Img(132,312:(312+127)) = ramp;
Img(13,312:(312+127)) = ramp;

imshow(Img)