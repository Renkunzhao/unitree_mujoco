#include <mujoco/mujoco.h>
#include <cmath>
#include <cstdio>
#include <memory>
int main(int argc, char** argv) {
  for (int arg = 1; arg < argc; ++arg) {
    char error[2048]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m(mj_loadXML(argv[arg], nullptr, error, sizeof(error)), mj_deleteModel);
    if (!m) { std::fprintf(stderr, "FAIL %s: %s\n", argv[arg], error); return 1; }
    std::unique_ptr<mjData, decltype(&mj_deleteData)> d(mj_makeData(m.get()), mj_deleteData);
    if (!d) return 2;
    for (int step = 0; step < 1000; ++step) {
      const double previous = d->time;
      mj_step(m.get(), d.get());
      if (d->time <= previous) { std::fprintf(stderr, "Time reset: %s\n", argv[arg]); return 3; }
      for (int i = 0; i < m->nq; ++i) if (!std::isfinite(d->qpos[i])) return 4;
      for (int i = 0; i < m->nv; ++i) if (!std::isfinite(d->qvel[i])) return 5;
      for (int i = 0; i < mjNWARNING; ++i) if (d->warning[i].number) {
        std::fprintf(stderr, "Warning %d: %s\n", i, argv[arg]); return 6;
      }
    }
    std::printf("PASS MuJoCo %s nq=%lld nv=%lld flex=%lld time=%.3f %s\n", mj_versionString(), (long long)m->nq, (long long)m->nv, (long long)m->nflex, d->time, argv[arg]);
  }
}
