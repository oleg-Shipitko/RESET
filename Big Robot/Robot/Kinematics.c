  float MLineSpeed[4][3] = { 1.0,  0.0, 0.0,  // матрица расчета линейных скоростей
                             0.0,  1.0, 0.0,
                             0.0, -1.0, 0.0,
                             -1.0,  0.0, 0.0};
  float MRotSpeed[4][3] = { 0.0, 0.0, -0.14 , // матрица расчета линейных скоростей
                            0.0, 0.0, -0.14,
                            0.0, 0.0, -0.14,
                            0.0, 0.0, -0.14};

  float InverseKinematics[4][4] =               // Матрица для обратного преобразования скоростей
  {0.50, 0.0, 0.0,-0.5,
   0.0, 0.5, -0.5,0.0,
   -25.0/14.0, -25.0/14.0, -25.0/14.0,-25.0/14.0,
   1, -1, -1,1};
