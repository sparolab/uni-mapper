#include <assert.h>
#include <omp.h>

int main(void) {
  int sum = 0;
  #pragma omp parallel for num_threads(4) reduction(+:sum)
  for (int i = 0; i < 1000; ++i) sum += 1;
  assert(sum == 1000);
#ifdef INJECT_RACE
  // The same runtime options must still detect application data races.
  #pragma omp parallel num_threads(4) shared(sum)
  sum += omp_get_thread_num();
#endif
  return 0;
}
