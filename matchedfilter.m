function Y = matchedfilter(X)

H_target = [
   0.0487 
  -0.1283 
  -0.1200  
   0.2913    
   0.1990   
  -0.5646   
  -0.1674    
   0.8045    
   0.1614   
  -0.9717   
  -0.0696    
   1.0000    
   0.2143   
  -0.8394   
  -0.1224    
   0.7399    
   0.0461
  -0.3810    
   0.0051
];

H_target = H_target/max(abs(H_target));
Y = filter(H_target,1,X);
  

