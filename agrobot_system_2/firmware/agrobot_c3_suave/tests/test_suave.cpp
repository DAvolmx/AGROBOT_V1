// Imprime mezclar() y rampa() del firmware real para compararlos con el port de Python.
// Uso:  g++ -O2 -std=c++17 -I.. test_suave.cpp -o /tmp/test_suave && /tmp/test_suave < casos.txt
#include <cstdio>
#include "suave_model.h"
int main() {
  suave::Lim L; double v, w, adel, atras, piv, minp, acel, frena, tras, cur, t;
  char modo;
  printf("%d\n", (int)suave::limValido(suave::Lim{}));   // los valores de fabrica DEBEN ser validos
  while (scanf(" %c", &modo) == 1) {
    if (modo == 'M') {            // M v omega adel atras pivote minp tras
      scanf("%lf %lf %lf %lf %lf %lf %lf", &v, &w, &adel, &atras, &piv, &minp, &tras);
      L.adel = adel; L.atras = atras; L.pivote = piv; L.minp = minp; L.tras = tras;
      auto o = suave::mezclar((float)v, (float)w, L);
      printf("%.5f %.5f %.5f %.5f\n", o.d[0], o.d[1], o.d[2], o.d[3]);
    } else {                      // R cur target minp acel frena
      scanf("%lf %lf %lf %lf %lf", &cur, &t, &minp, &acel, &frena);
      L.minp = minp; L.acel = acel; L.frena = frena;
      printf("%.5f\n", suave::rampa((float)cur, (float)t, 0.01f, L));
    }
  }
}
