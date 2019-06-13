'''using aruco markers for visual servoing, we will get the image from gazebo camera
   then aruco detection from opencv is used to get 4 corners of the marker
   the desired pose is in for of image  in utils.
   We assume that the camera parameters is known and images are rectifed of distortion.
   We will assign a generic depth to the marker and try to move the robot to attain that depth.
   for issues mail to haritpandya@gmail.com

   note the suffix <.>d is used to represent the desired configuration
   and <.>cur and <.>init configuration 
''' 

import sys
import numpy as np
import cv2
import time

# Add the utils folder path to the sys.path list
sys.path.append('../utils')
from get_aruco_keypoints import getkeypts


#we assume initial camera pose and current camera pose as identity
Tinit = eye(4)
Tcurr=Tinit

#desired pose could be given in form of xy coordinates 
#xydes=np.array([0 0; 200 0; 0 200; 200 200])

#Alternatively, desired pose could be given in form of image 
#imgd = cv2.imread('../utils/test_marker.jpg')

#Another option is to move the camera to a desired pose and get image.
#Tf = Tinit * [[eul2rotm([0,0,0]) [0.4;0;0]]; 0 0 0 1];
#status = gazebo_movecam(gazebo_handle ,Tf)
#imgd = gazebo_getimage(gazebo_handle);
#status = gazebo_movecam(gazebo_handle ,Tcurr)

kp_des=tf.flatten(xydes);

#ibvs tuning parameters
lambda=0.0001;
desired_depth=10;



if __name__== "__main__":
    img = cv2.imread('../utils/test_marker.jpg')
    corners = getkeypts(img)
    iter = 1
    while(1):
        img = gazebo_getimage(gazebo_handle);
        xy_curr = (find_centre(img));
        kp_curr=xy_curr(:);
        img1=img;
        #displaying detected centeres
        img = insertShape(img,'FilledCircle',[xy_curr(1,1), xy_curr(2,1), 5],'Color',[1,1,0]);
        img = insertShape(img,'FilledCircle',[xy_curr(1,2), xy_curr(2,2), 5],'Color',[1,1,0]);
        img = insertShape(img,'FilledCircle',[xy_curr(1,3), xy_curr(2,3), 5],'Color',[1,1,0]);
        img = insertShape(img,'FilledCircle',[xy_curr(1,4), xy_curr(2,4), 5],'Color',[1,1,0]);

        #displaying desired centeres
        img = insertShape(img,'FilledCircle',[xydes(1,1), xydes(2,1), 5],'Color',[1,0,0]);
        img = insertShape(img,'FilledCircle',[xydes(1,2), xydes(2,2), 5],'Color',[1,0,0]);
        img = insertShape(img,'FilledCircle',[xydes(1,3), xydes(2,3), 5],'Color',[1,0,0]);
        img = insertShape(img,'FilledCircle',[xydes(1,4), xydes(2,4), 5],'Color',[1,0,0]);

        imshow(img);
    
    
        #getinteraction_intensity(<current keypoints>,<camera.intensic>,<length
        #of featurevector>,<estimate of camera depth if true depth not available>,
        #<flag to use true depth (1) or estmiate(0)>,<true depth of every keypoint>);
        Lsd=getinteraction_point(kp_curr,gazebo_handle.cam,length(kp_curr),desired_depth,0,desired_depth);
    
        error=kp_curr-kp_des;
        vc=-lambda*pinv(Lsd)*error;
        fprintf('error=%.2f, vc=%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n',norm(error),vc(1),vc(2),vc(3),vc(4),vc(5),vc(6));
    
        #vc= [0.1 0 0 0 0 0];
        if(iter > 200):
            break; 
        Rotd = eul2rotm([vc(4), vc(5), vc(6)]);%ZYX
        Td = [Rotd [vc(1);vc(2);vc(3)]];
        Tcurr=Tcurr*[Td;0 0 0 1];
        status = gazebo_movecam(gazebo_handle ,Tcurr);
        iter = iter +1;





