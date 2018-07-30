case mot_turn:
       //printf("ref_angle %f\n", ref_angle);
       angle_cal = (p->right_pos-p->left_pos)/p->w;
       p->motorspeed_l =-reg_k*(ref_angle-angle_cal);
       p->motorspeed_r =reg_k*(ref_angle-angle_cal);
        //printf("motorspeed_l %f, motorspeed_r %f and angle %f\n",p->motorspeed_l,p->motorspeed_r,angle_cal);
       if (angle_cal >= ref_angle*0.982){
	 p->motorspeed_r = 0;
	 p->motorspeed_l = 0;
         p->finished=1;
       }