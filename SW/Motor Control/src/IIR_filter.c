#include "IIR_filter.h"

float IIR(float x_0, float y_1, float decaimento)
{
    return (1 - decaimento) * x_0 + decaimento * y_1;
}

// xFiltrado = (1 - decaimento) * a_x + decaimento * xFiltrado;
// yFiltrado = (1 - decaimento) * a_y + decaimento * yFiltrado;
// zFiltrado = (1 - decaimento) * a_z + decaimento * zFiltrado;

// roll = atan2(yFiltrado, zFiltrado) * 180.0 / PI;
// roll = roll + rollOffset;
// pitch = atan2(-xFiltrado, sqrt(yFiltrado * yFiltrado + zFiltrado * zFiltrado)) * 180.0 / PI;
// pitch = pitch + pitchOffset;

// ****************************** filtro direto nos angulos ****************************** //
// rollFiltrado = (1 - decaimento) * roll + decaimento * rollFiltrado;
// pitchFiltrado = (1 - decaimento) * pitch + decaimento * pitchFiltrado;