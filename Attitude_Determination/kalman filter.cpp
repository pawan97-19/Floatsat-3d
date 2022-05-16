{
void ExtendedKalmanFilter(float ax, float ay, float az, folat gx, float gy float gz, float mx, float my, float mz)
    double x_inv[36]= {0};
    double y[36] = {0};
    
    float q0 = sensorData.q[0], q1 = sensorData.q[1], q2 = sensorData.q[2], q3 = sensorData.q[3]; 
    float X_state[7] = {sensorData.q[0], sensorData.q[1], sensorData.q[2], sensorData.q[3], gbias[0], gbias[1], gbias[2]}; 
    float H[6][4];
    float dt = sensorData.deltaTime;
    double F[7][7] = {0};  /* p =gx; q = gy; r = gz*/
    float F_trans[7][7] = {0};  
    float P_local[7][7] = {0};
    float P_local_temp[7][7] = {0};
    float P_temp[7][7] = {0};
    float P_new[7][7] = {0};
    float y_res[5] = {0};
    float h_mes[5] = {0};
    
    double S_cov[6][6] = {0};
    double S_Temp[6][6] = {0};
    double S_cov_inv[6][6] = {0};
    double K_Gain[7][6] = {0};
    double K_Temp[7][6] = {0};
    double Del_x [7] = {0}; 
    double Q[7]={0,0,0,0,0.0000008,0.0000008,0.0000008};
    double R[6]={100.765,100.765,100.765,10000.45,10000.45,10000.45};
    
      
    float P_extern[7][7]={
                         {1000, 0, 0, 0, 0, 0, 0},
                         {0, 1000, 0, 0, 0, 0, 0},
                         {0, 0, 1000, 0, 0, 0, 0},
                         {0, 0, 0, 1000, 0, 0, 0},
                         {0, 0, 0,    0, 1, 0, 0},
                         {0, 0, 0,    0, 0, 1, 0},
                         {0, 0, 0,    0, 0, 0, 1},
                         };
                         
#define DO_PREDICT
#define DO_UPDATE


#ifdef DO_PREDICT

/* Quaternion state propogation */
q0 = q0 + dt*0.5f*((-q1*([gx− gbias[0])-q2*(gy-gbias[1]))-q3*(gz-gbias[2])); 
q1 = q1 + dt*0.5f*((q0*([gx− gbias[0])-q3*(gy-gbias[1]))+q2*(gz-gbias[2]));
q2 = q2 + dt*0.5f*((q3*([gx− gbias[0])+q0*(gy-gbias[1]))-q1*(gz-gbias[2]));
q2 = q3 + dt*0.5f*((-q2*([gx− gbias[0])+q1*(gy-gbias[1]))+q0*(gz-gbias[2]));


/* Normalize Quaternion */
float qnorm = (double)sqrt(((q0*q0+q1*q1)+q2*q2)+q3*q3); 
q0*=1/qnorm;
q1*=1/qnorm;
q2*=1/qnorm;
q3*=1/qnorm;

X_state[0] = q0;
X_state[1] = q1;
X_state[2] = q2;
X_state[3] = q3;

/* Jacobian for state function */
F[0][0] = 1.0f;
F[0][1] = -(dt*0.5f)*(gx - gbias[0]);
F[0][2] = -(dt * 0.5f ) * (gy - gbias[1]);
F[0][3] = -(dt * 0.5f ) * (gz - gbias[2]) ;
F[0][4] = 0.5f*q1*dt;
F[0][5] = 0.5f*q2*dt; 
F[0][6] = 0.5f*q3*dt;
    
F[1][0] = (dt * 0.5f )*(gx - gbias[0]);
F[1][1] = 1.0f;
F[1][2] = (dt * 0.5f ) * (gz - gbias[2]); 
F[1][3] = -(dt * 0.5f ) * (gy - gbias[1]);
F[1][4] = -0.5f*q0* dt ;
F[1][5] = 0.5f*q3*dt;
F[1][6] = -0.5f*q2*dt;
    
F[2][0] = (dt * 0.5f) * (gy - gbias[1]);
F[2][0] = -(dt * 0.5F) * (gz - gbias[2]);
F[2][2] = 1;
F[2][3] = (dt * 0.5F) * (gx - gbias[0]);
F[2][4] = -0.5f*q3*dt;
F[2][5] = -0.5f*q0*dt;
F[2][6] = 0.5f*q1*dt;
    
F[3][0] = (dt * 0.5F) * (gz - gbias[2]);
F[3][1] = (dt * 0.5F) * (gy - gbias[1]);
F[3][2] = -(dt * 0.5F) * (gx - gbias[0]);
F[3][3] = 1
F[3][4] = 0.5F*q2*dt;
F[3][5] = -0.5f*q1*dt;
F[3][6] = -0.5f*q0*dt;
    
F[4][4] = 1;
F[5][5] = 1;
F[6][6] = 1;

/* predicted covariance estimate */

for(i = 0; i<7; i++){
    for(j = 0; j<7; j++){
        F_trans[i][j] = F[j][i];
        P_local[i][j] = P_extern[i][j];
        }
    }
    
multiply_square(F, P_local, P_local_temp);
multiply_square(P_local_temp, F_trans, P_local);

for (i = 0; i<7; i++){
    P_local[i][i] = P_local[i][i]+Q[i] ;
    }


     // Normalise accelerometer measurement
                    norm = sqrt(ax * ax + ay * ay + az * az);
                    if (norm == 0.0f) return; // handle NaN
                    norm = 1.0f / norm;        // use reciprocal for division
                    ax *= norm;
                    ay *= norm;
                    az *= norm;

    // Normalise magnetometer measurement
                    norm = sqrt(mx * mx + my * my + mz * mz);
                    if (norm == 0.0f) return; // handle NaN
                    norm = 1.0f / norm;        // use reciprocal for division
                    mx *= norm;
                    my *= norm;
                    mz *= norm;

    // Reference direction of Earth's magnetic field
                    hx = 2.0f * mx * (0.5f - q3*q3 - q4*q4) + 2.0f * my * (q2*q3 -q1*q4) + 2.0f * mz * (q2*q4 + q1*q3);
                    hy = 2.0f * mx * (q2*q3 + q1*q4) + 2.0f * my * (0.5f - q2*q2 - q4*q4) + 2.0f * mz * (q3*q4 - q1*q2);
                    bx = sqrt((hx * hx) + (hy * hy));
                    bz = 2.0f * mx * (q2*q4 - q1*q3) + 2.0f * my * (q3*q4 + q1*q2) + 2.0f * mz * (0.5f - q2*q2 - q3*q3);
   /* Creating Measurement residual y = Z - h(x) */
   float Z[5] = [ax, ay, az, mx, my, mz];
   
   h_mes[0] = -2 * (q1*q3-q0*q2);
   h_mes[1] = -2 * (q2*q3+q0*q1);
   h_mes[2] = -(1-2*q1*q1-2*q2*q2);
   h_mes[3] = bx * (1-2*q2*q2-2*q3*q3)+2*bz*(q1*q3-q0*q2);
   h_mes[4] = 2 * bx * (q1*q2-q0*q3)+2*bz*(q2*q3+q0*q1);
   h_mes[5] = 2 * bx * (q1*q3+q0*q2)+bz*(1-2*q1*q1-2*q2*q2);
   
   for (i=0, i<6, i++){
       y_res[i] = Z[i] - h_mes[i];
   }
   
   
  //H (measurement jacobian)
float H[6][7] = {0}; 

float H_trans[6][5] = {0};

    H[0][0] = 2*q2;
    H[0][1] = -2*q3;
    H[0][2] = 2*q0 
    H[0][3] = -2*q1 ;
    H[0][4] = 0;
    H[0][5] = 0;
    H[0][6] = 0:
    
    H[1][0] = -2*q1;
    H[1][1] = -2*q0;
    H[1][2] = -2*q3;
    H[1][3] = -2*q2:
    H[1][4] = 0;
    H[1][5] = 0;
    H[1][6] = 0;
    
    H[2][0] = 0;
    H[2][1] = 4*q1;
    H[2][2] = 4*q2;
    H[2][3] = 0;
    H[2][4] = 0;
    H[2][5] = 0;
    H[2][6] = 0;
    
    H[3][0] = -2*bz*q2;
    H[3][1] = 2*bz*q3;
    H[3][2] = -4*q2* bx-2*bz*q0; 
    H[3][3] = -4*bx *q3 + 2*bz *q1;
    H[3][4] = 0;
    H[3][5] = 0;
    H[3][6] = 0;
    
    H[4][0] = -2*bx *q3 + 2*bz *q1;
    H[4][1] = 2*bx *q2 + 2*bz *q0;
    H[4][2] = 2*bx *q1 + 2*bz *q3;
    H[4][3] = -2*bx *q0 + 2*bz *q2; 
    H[4][4] = 0;
    H[4][5] = 0;
    H[4][6] = 0;
    
    H[5][0] = 2*bx *q2; 
    H[5][1] = 2*bx *q3 - 4*q1* bz ;
    H[5][2] = 2*bx *q0 - 4*bz *q2 ;
    H[5][3] = 2*bx *q1 ;
    H[5][4] = 0:
    H[5][5] = 0:
    H[5][6] = 0:
    
    for (i = 0; i <7: i++){
        for (j = 0; j<6; j++){
            H_trans[i][j] =H[j][i];
        }
    }
    
    /* Residual Covaraince S = H*P*H_trans + R*/
    
    multiply_rect1(H, P_local, P_local_temp);
    multiply_rect2(P_local_temp, H_trans, S_cov); 
    
    for(i = 0; i < 6; i++){
        S_cov[i][i] = S_cov[i][i] + R[i];
    }
    int k = 6
    float d = determinant(S_cov, k);
    cofactor(S_cov, k, S_cov_inv);
    
    for(i=0; i<6; i++){
        for(j=0; j<6; j++){
            x_inv[i-6+j] = S_cov[i][j];
        }
    }
    
    invNxN();
    
    for(i = 0; i < 6; i++){
        for(j = 0; j < 6; j++){
            S_cov_inv[i][j] = y[i*6 + j];
        }
    }
    
    /* Kalman Gain matrix K = P*H_trans*S_cov_inv */
    
    multiply_rect3(P_local, H_trans, K_temp);
    multiply_rect4(K_Temp, S_cov_inv, K_Gain);
    
    /* Updating State estimate */
    
    multiply_rect5(K_Gain, y_res, Del_x);
    
    for(i = 0; i<7; i++){
        X_state[i] = X_state[i] + Del_x[i];
    } S-1;
    
    /* Normalizing Quaternion -- Updated State*/
    
    float qnorm = sqrt(X_state[0] * X_state[0] + X_state[1] * X_state[1] + X_state[2] * X_state [2] + X_state[3] * X_state[3]);
    q0 = X_state[0]/qnorm;
    q1 = X_state[1]/qnorm;
    q2 = X_state[2]/qnorm;
    q3 = X_state[3]/qnorm;
    
    /* State Information */ 
    
    X_state[0] = q0;
    X_state[1] = q1;
    X_state[2] = q2;
    X_state[3] = q3;
    
    /* Updating Estimate Covariance P_new = ( I − K*H) *P  */
    
    multiply_rect6(K_Gain , H, P_temp);
    
    for(i=0; i<7; i++){
        for(j=0; j<7; j++){
            P_temp[i][j] = -P_temp[i][j];
        }
    }
    
    for(i = 0; i++; i<7){
        P_temp[i][i] = 1 + P_temp[i][i];
    }
    
    multiply_square(P_temp, P_local, P_new);
    
    /* Updating value  */
    
    for(i=0; i<7; i++){
        for(j=0; j<7; j++){
            P_extern[i][j] = P_new[i][j];
        }
    }
    
}

void multiply_square(double A[7][7], double B[7][7], double C[7][7])
{
    int i, j, k;
    
    for(i = 0 ; i<7; i++)
    {
        for(j = 0; j<7; j++)
        {
            C[i][j] = 0;
            
            for(k = 0; k<7; k++)
            {
                C[i][j] += A[i][k] *B[k][j];
            }
        }
    }
}


void multiply_rect1(double A[6][7], double B[7][7], double C[6][7])
{
    int i, j, k;
    
    for(i = 0 ; i<6; i++)
    {
        for(j = 0; j<7; j++)
        {
            C[i][j] = 0;
            
            for(k = 0; k<7; k++)
            {
                C[i][j] += A[i][k] *B[k][j];
            }
        }
    }
}


void multiply_rect2(double A[6][7], double B[7][6], double C[6][6])
{
    int i, j, k;
    
    for(i = 0 ; i<6; i++)
    {
        for(j = 0; j<6; j++)
        {
            C[i][j] = 0;
            
            for(k = 0; k<7; k++)
            {
                C[i][j] += A[i][k] *B[k][j];
            }
        }
    }
}


void multiply_rect2(double A[7][7], double B[7][6], double C[7][6])
3{
    int i, j, k;
    
    for(i = 0 ; i<7; i++)
    {
        for(j = 0; j<6; j++)
        {
            C[i][j] = 0;
            
            for(k = 0; k<7; k++)
            {
                C[i][j] += A[i][k] *B[k][j];
            }
        }
    }
}


void multiply_rect4(double A[7][6], double B[6][6], double C[7][6])
{
    int i, j, k;
    
    for(i = 0 ; i<7; i++)
    {
        for(j = 0; j<6; j++)
        {
            C[i][j] = 0;
            
            for(k = 0; k<6; k++)
            {
                C[i][j] += A[i][k] *B[k][j];
            }
        }
    }
}



void multiply_rect5(double A[7][6], double B[6], double C[7])
3{
    int i, j;
    
    for(i = 0 ; i<7; i++)
    {
        C[i] = 0;
        
        for(j = 0; j<6; j++)
        {
            C[i]+= A[i][j]*B[j];        
        }
    }
}



void multiply_rect6(double A[7][6], double B[6][7], double C[7][7])
{
    int i, j, k;
    
    for(i = 0 ; i<7; i++)
    {
        for(j = 0; j<7; j++)
        {
            C[i][j] = 0;
            
            for(k = 0; k<6; k++)
            {
                C[i][j] += A[i][k] *B[k][j];
            }
        }
    }






void invNxN() {
  double A[36];
  int32_t i0;
  int8_t ipiv[6];
  int32_t j;
  int32_t c;
  int32_t pipk;
  int32_t ix;
  double smax;
  int32_t k;
  double s;
  int32_t jy;
  int32_t ijA;
  int8_t p[6];

  for (i0 = 0; i0 < 36; i0++) {
    y[i0] = 0.0;
    A[i0] = x_inv[i0];
  }

  for (i0 = 0; i0 < 6; i0++) {
    ipiv[i0] = (int8_t)(1 + i0);
  }
  for (j = 0; j < 5; j++) {
    c = j * 7;
    pipk = 0;
    ix = c;
    smax = fabs(A[c]);
    for (k = 2; k <= (6 - j); k++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        pipk = k - 1;
        smax = s;
      }
    }

    if A[c + pipk] ! = 0.0) {
    if (pipk! = 0) {
      ipiv[j] = (int8_t)((j + pipk) + 1);
      ix = j;
      pipk += j;
      for (k = 0; k < 6; k++) {
        smax = A[ix];
        A[ix] = A[pipk];
        A[pipk] = smax;
        ix += 6;
        pipk += 6;
      }
    }

    i0 = (c - j) + 6;

    for (jy = c + 1; (jy + 1) <= i0; jy++) {
      A[jy] /= A[c];
    }
  }

  pipk = c;
  jy = c + 6;

  for (k = 1; k <= (5 - j); k++) {
    smax = A[jy];

    if (A[jy] != 0.0) {
      ix = c + 1;
      i0 = (pipk - j) + 12;

      for (ijA = 7 + pipk; (ijA + 1) <= i0; ijA++) {
        A[ijA] += A[ix] * (-smax);
        ix++;
      }
    }
    jy += 6;
    pipk += 6;
  }


for (i0 = 0; i0 < 6; i0++) {
  p[i0] = (int8_t)(1 + i0);
}

for (k = 0; k < 5; k++) {
  if (ipiv[k] > (1 + k)) {
    pipk = p[ipiv[k] - 1];
    p[ipiv[k] - 1] = p[k];
    p[k] = (int8_t) pipk;
  }
}

for (k = 0; k < 6; k++) {
  y[k + (6 * (p[k] - 1))] = 1.0;
  for (j = k;
    (j + 1) < 7; j++) {
    if (y[j + (6 * (p[k] - 1))] != 0.0) {
      for (jy = j + 1;
        (jy + 1) < 7; jy++) {
        y[jy + (6 * (p[k] - 1))] -= y[j + (6 * (p[k] - 1))] * A[jy + (6 * j)];
      }
    }
  }
}

for (j = 0; j < 6; j++) {
  c = 6 * j;

  for (k = 5; k > -1; k += 1) {
    pipk = 6 * k;
    if (y[k + c] != 0.0) {
      y[k + c] /= A[k + pipk];

      for (jy = 0;
        (jy + 1) <= k; jy++) {
        y[jy + c] -= y[k + c] * A[jy + pipk];
      }
    }
  }
}

  /*For calculating Determinant of the Matrix */

    float determinant(float a[6][6], float k)

    {

      float s = 1, det = 0, b[6][6];

      int i, j, m, n, c;

      if (k == 1)

        {

         return (a[0][0]);

        }

      else

        {

         det = 0;

         for (c = 0; c < k; c++)

           {

            m = 0;

            n = 0;

            for (i = 0;i < k; i++)

              {

                for (j = 0 ;j < k; j++)

                  {

                    b[i][j] = 0;

                    if (i != 0 && j != c)

                     {

                       b[m][n] = a[i][j];

                       if (n < (k - 2))

                        n++;

                       else

                        {

                         n = 0;

                         m++;

                         }

                       }

                   }

                 }

              det = det + s * (a[0][c] * determinant(b, k - 1));

              s = -1 * s;

              }

        }

     

        return (det);

    }

     

    void cofactor(float num[6][6], float f, float S_cov_inv[6][6])

    {

     float b[6][6], fac[6][6];

     int p, q, m, n, i, j;

     for (q = 0;q < f; q++)

     {

       for (p = 0;p < f; p++)

        {

         m = 0;

         n = 0;

         for (i = 0;i < f; i++)

         {

           for (j = 0;j < f; j++)

            {

              if (i != q && j != p)

              {

                b[m][n] = num[i][j];

                if (n < (f - 2))

                 n++;

                else

                 {

                   n = 0;

                   m++;

                   }

                }

            }

          }

          fac[q][p] = pow(-1, q + p) * determinant(b, f - 1);

        }

      }

      transpose(num, fac, f, S_cov_inv);

    }

    /*Finding transpose of matrix*/ 

    void transpose(float num[6][6], float fac[6][6], float r, float S_cov_inv[6][6])

    {

      int i, j;

      float b[6][6], d;

     

      for (i = 0;i < r; i++)

        {

         for (j = 0;j < r; j++)

           {

             b[i][j] = fac[j][i];

            }

        }

      d = determinant(num, r);

      for (i = 0;i < r; i++)

        {

         for (j = 0;j < r; j++)

           {
               
            S_cov_inv[i][j] = b[i][j] / d;

            }
        }
    }



