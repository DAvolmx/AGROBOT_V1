// M v w adel atras pivote minp tras dif_del boost | R cur t minp acel frena | B env t_ms periodo
#include <cstdio>
#include "fusion_model.h"
int main() {
  fusion::Lim L; char modo; double a[12];
  printf("%d\n", (int)fusion::limValido(fusion::Lim{}));
  while (scanf(" %c", &modo) == 1) {
    if (modo == 'M') {
      for (int i = 0; i < 9; i++) scanf("%lf", &a[i]);
      L.adel = a[2]; L.atras = a[3]; L.pivote = a[4]; L.minp = a[5]; L.tras = a[6]; L.dif_del = a[7];
      auto o = fusion::mezclar((float)a[0], (float)a[1], L, (float)a[8]);
      printf("%.5f %.5f %.5f %.5f\n", o.d[0], o.d[1], o.d[2], o.d[3]);
    } else if (modo == 'G') {            // G v w adel atras pivote minp tras dif_del comp_neg comp_pos
      for (int i = 0; i < 10; i++) scanf("%lf", &a[i]);
      L.adel = a[2]; L.atras = a[3]; L.pivote = a[4]; L.minp = a[5]; L.tras = a[6]; L.dif_del = a[7]; L.comp_neg = a[8]; L.comp_pos = a[9];
      auto o = fusion::mezclar((float)a[0], (float)a[1], L, 1.0f);
      float e[4]; fusion::escala_giro((float)a[1], o, L, e);
      printf("%.5f %.5f %.5f %.5f\n", e[0], e[1], e[2], e[3]);
    } else if (modo == 'R') {
      for (int i = 0; i < 5; i++) scanf("%lf", &a[i]);
      L.minp = a[2]; L.acel = a[3]; L.frena = a[4];
      printf("%.5f\n", fusion::rampa((float)a[0], (float)a[1], 0.01f, L));
    } else {
      for (int i = 0; i < 3; i++) scanf("%lf", &a[i]);
      L.periodo = a[2];
      printf("%.5f\n", fusion::rafaga((float)a[0], (uint32_t)a[1], L));
    }
  }
}
