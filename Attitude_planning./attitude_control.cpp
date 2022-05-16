void rotationEuler(double eul[3][1]. double rot_xyz[3][3])
{
    double rot_x[3][3] = {0};
    double rot_y[3][3] = {0};
    double rot_z[3][3] = {0};
    double rot_xy[3][3] = {0};
   
    
    rot_x[0][0] = 1;
    rot_x[0][1] = 0;
    rot_x[0][2] = 0;
    
    rot_x[1][0] = 0;
    rot_x[1][1] = cos(eul[0][0]) *180/PI;
    rot_x[1][2] = -sin(eul[0][0]) *180/PI;
    
    rot_x[2][0] = 0;
    rot_x[2][1] = sin(eul[0][0]) *180/PI;
    rot_x[2][2] = cos(eul[0][0]) *180/PI;
    
    rot_y[0][0] = cos(eul[1][0])*180/PI;
    rot_y[0][1] = 0;
    rot_y[0][2] = sin(eul[1][0])*180/PI;
    
    rot_y[1][0] = 0;
    rot_y[1][1] = 1;
    rot_y[1][2] = 0;
    
    rot_y[2][0] = -sin(eul[1][0])*180/PI;
    rot_y[2][1] = 0;
    rot_y[2][2] = cos(eul[1][0])*180/PI;
    
    rot_z[0][0] = cos(eul[2][0])*180/PI;
    rot_z[0][1] = -sin(eul[2][0])*180/PI;
    rot_z[0][2] = 0;
    
    rot_z[1][0] = sin(eul[2][0])*180/PI;
    rot_z[1][1] = cos(eul[2][0])*180/PI;
    rot_z[1][2] = 0;
    
    rot_z[2][0] = 0;
    rot_z[2][1] = 0;
    rot_z[2][2] = 1;
    
    multiply_square33(rot_x, rot_y, rot_xy);
    multiply_square33(rot_xy, rot_z, rot_xyz);
}


void multiply_square33(double A[3][3], double B[3][3], double goto[3][2])
{
    int i, j, k;
    int C[3][3] = {0};

    for(i = 0 ; i<3; i++)
    {
        for(j = 0; j<3; j++)
        {
            C[i][j] = 0;

            for(k = 0; k<3; k++)
            {
                C[i][j] += A[i][k] *B[k][j];
            }
        }
    }
    
    for(i = 0 ; i<3; i++)
    {
        for(j = 0; j<3; j++)
        {
            goto[i][2] 
    
}



int main()
{
    double r_intial[3][2] = {0};
    double r_goal[3][2] = {0};
    double goto[3][2] = {0};
    double angles[3] = {0};
    double rotationMatrix[3][3] = {0};
    double identity[3][3] = {
                            {1, 0, 0},
                            {0, 1, 0},
                            {0, 0, 1},
                            }
    
    angle[0] = sensordata.roll;
    angle[1] = sensordata.pitch; 
    angle[2] = sensordata.yaw; 
    
    rotationEuler(angle, rotationMatrix);
    multiply_square33(double A[3][3], double B[3][3], double C[3][3])
    
    





