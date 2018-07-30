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
  for(cnt = 0; cnt < 8; cnt++) {
    linearr[cnt] = (linesensor->data[cnt]-BLACK_CALIB)*1.0/(WHITE_CALIB-BLACK_CALIB);
  }
}

float lowest_line() {
  int low = 0;
  float retu = 1;
  line_transform();
  for(cnt = 0; cnt < 8; cnt++) {
    if (linearr[cnt] < retu) {
      low = cnt;
      retu = linearr[cnt];
    }
  }
  switch (low) {
    case 0:
      retu = -0.3218;
      break;
    case 1:
      retu = -0.2337;
      break;
    case 2:
      retu = -0.1419;
      break;
    case 3:
      retu = -0.0476;
      break;
    case 4:
      retu = 0.0476;
      break;
    case 5:
      retu = 0.1419;
      break;
    case 6:
      retu = 0.2337;
      break;
    case 7:
      retu = 0.3218;
      break;
  }
   return retu;
}

