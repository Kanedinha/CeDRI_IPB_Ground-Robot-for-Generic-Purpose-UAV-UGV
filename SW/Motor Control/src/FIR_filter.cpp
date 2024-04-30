#include <FIR_filter.h>

float filtroFIR(float buffer[], int indice) {
    float resultado = 0.0;
    for (int i = 0; i < NUM_COEFICIENTES; ++i) {
        resultado += coeficientes[i] * buffer[i];
    }
    return resultado;
}



  // bufferX[indice] = a_x;
  // bufferY[indice] = a_y;
  // bufferZ[indice] = a_z;
  // xFiltrado = 0;
  // yFiltrado = 0;
  // zFiltrado = 0;
  // for (int i = 0; i < NUM_COEFICIENTES; i++)
  // {
  //   int indiceBuffer = (indice + i) % NUM_COEFICIENTES;
  //   xFiltrado += coeficientes[i] * bufferX[indiceBuffer];
  //   yFiltrado += coeficientes[i] * bufferY[indiceBuffer];
  //   zFiltrado += coeficientes[i] * bufferZ[indiceBuffer];
  // }
  // indice = (indice + 1) % NUM_COEFICIENTES;

  // roll = atan2(yFiltrado, zFiltrado) * 180.0 / PI;
  // roll = roll + rollOffset;
  // pitch = atan2(-xFiltrado, sqrt(yFiltrado * yFiltrado + zFiltrado * zFiltrado)) * 180.0 / PI;
  // pitch = pitch + pitchOffset;

  // bufferX[indice] = a_x;
  // bufferY[indice] = a_y;
  // bufferZ[indice] = a_z;

  // xFiltrado = FIR(bufferX, indice);
  // yFiltrado = FIR(bufferY, indice);
  // zFiltrado = FIR(bufferZ, indice);

  // indice = (indice + 1) % NUM_COEFICIENTES;