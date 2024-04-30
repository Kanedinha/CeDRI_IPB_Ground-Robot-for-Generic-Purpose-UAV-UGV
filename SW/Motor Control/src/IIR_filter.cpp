#include "IIR_filter.h"

// Coeficientes do filtro IIR
float b[] = {0.1220, 0.5502, 1.0467, 1.0467, 0.5502, 0.1220};
float a[] = {1, 0.9143, 0.9808, 0.4013, 0.1271, 0.0143};

// Tamanho dos coeficientes
int N = sizeof(b) / sizeof(b[0]);

// Buffer de entrada
float x[6] = {0};

// Buffer de saída
float y[6] = {0};

// Função para aplicar o filtro IIR
float filtroIIR(float entrada) {
    // Atualiza o buffer de entrada
    for (int i = N - 1; i > 0; i--) {
        x[i] = x[i - 1];
    }
    x[0] = entrada;

    // Calcula a saída do filtro
    float saida = 0;
    for (int i = 0; i < N; i++) {
        saida += b[i] * x[i] - a[i] * y[i];
    }

    // Atualiza o buffer de saída
    for (int i = N - 1; i > 0; i--) {
        y[i] = y[i - 1];
    }
    y[0] = saida;

    return saida;
}

// ******************** filtro IIR ordem 1 ******************** //
// float IIR(float x_0, float y_1, float decay)
// {
//     return (1 - decay) * x_0 + decay * y_1;
// }

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





