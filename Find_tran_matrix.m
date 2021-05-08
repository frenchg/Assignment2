function [Best_A] = Find_tran_matrix(InputImage1,InputImage2)
N_ransac = 10000; % number of ransac interations

image_1 = imread(InputImage1);
image_2 = imread(InputImage2);


Image1gray = rgb2gray(image_1);
Image2gray = rgb2gray(image_2);

im1 = im2double(Image1gray);
im2 = im2double(Image2gray);

points1 = detectSURFFeatures( im1 );
features1 = extractFeatures( im1,points1 );

points2 = detectSURFFeatures( im2 );
features2 = extractFeatures( im2,points2 );

indexPairs = matchFeatures( features1, features2, 'Unique', true );

matchedPoints1 = points1( indexPairs( :,1 ) );
matchedPoints2 = points2( indexPairs( :,2 ) );

im1_points = matchedPoints1.Location;
im2_points = matchedPoints2.Location;



P = [];

%im1_points = [1373,1204;1841,1102;1733,1213;2099,1297];
%im2_points = [182,1160;728,1055;617,1172;1001,1247];

Points_size = length(im1_points);
count = 0;
best_count = 0;


for j = 1:N_ransac
    if best_count<count
       best_count = count;
       best_corr = random_coord;
       Best_A = A; % updating the best A value
    end
    
    random_coord = randperm(2706,4);
    for i = 1:4  % creating a transform matrix
       count = 0;
       

       P(2*i-1,1) = -im1_points(random_coord(i),1);
       P(2*i-1,2) = -im1_points(random_coord(i),2);
       P(2*i-1,3) = -1;
       P(2*i-1,4) = 0;
       P(2*i-1,5) = 0;
       P(2*i-1,6) = 0;
       P(2*i-1,7) = (im1_points(random_coord(i),1)*im2_points(random_coord(i),1));
       P(2*i-1,8) = (im2_points(random_coord(i),1)*im1_points(random_coord(i),2));
       P(2*i-1,9) = im2_points(random_coord(i),1);


       P(2*i,1) = 0;
       P(2*i,2) = 0;
       P(2*i,3) = 0;
       P(2*i,4) = -im1_points(random_coord(i),1);
       P(2*i,5) = -im1_points(random_coord(i),2);
       P(2*i,6) = -1;
       P(2*i,7) = (im2_points(random_coord(i),2)*im1_points(random_coord(i),1));
       P(2*i,8) = (im2_points(random_coord(i),2)*im1_points(random_coord(i),2));
       P(2*i,9) = im2_points(random_coord(i),2);

    end
      [U,S,V] = svd(P);

       V_T = transpose(V);

       C = (V_T(9,:));
       
       A = transpose(reshape(C,3,3));% finding the transfrom from SVD

   for k = 1:Points_size % testing transform matrix on all points 

     corcheck = A*[im1_points(k,1);im1_points(k,2);1];
     X_prime = corcheck(1)/corcheck(3);
     Y_prime = corcheck(2)/corcheck(3);

     x_dif = (im2_points(k,1)- X_prime);
     y_dif =(im2_points(k,2)- Y_prime); %finding the distance in x and y

     e = sqrt(x_dif^2+y_dif^2);
        if e<10 % checking how good each transfrom for x y to x' y' is
            count = count+1;
           
        end
   end
end
end






