     case mot_move:   
       p->motorspeed_l =MAX_SPEED-reg_k*lowest_line();
       p->motorspeed_r =MAX_SPEED+reg_k*lowest_line();
       if ((p->right_pos+p->left_pos)/2- p->startpos > p->dist){
          p->finished=1;
	  p->motorspeed_l=0;
          p->motorspeed_r=0;
       }
     break;

void line_transform() {
  for(q = 0; q < 8; q++) {
    linearr[q] = (linesensor->data[q]-BLACK_CALIB)*1.0/(WHITE_CALIB-BLACK_CALIB);
  }
}

float lowest_line() {
  
  float dist[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
  float numerator=0;
  float denominator=0;
  
  line_transform();
  
  for(q = 0; q < 8; q++) {
    numerator += (1-linearr[q]) * dist[q];
    denominator += (1-linearr[q]);
  }
  
  return atan(numerator/denominator/21);  
}


