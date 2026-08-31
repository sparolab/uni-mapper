# OpenLMM Experiment Application

This wheel owns the `open-lmm-experiment` console command. Reusable experiment
models and execution logic remain in the exact-version `open-lmm` SDK wheel.

```bash
open-lmm-experiment validate --manifest experiment.json
open-lmm-experiment run --manifest experiment.json \
  --dataset-root /data --evidence-root /output/evidence
```

The supported release is OpenLMM 3.0.0 on CPython 3.10 in the reviewed local
Ubuntu 22.04 x86-64 image.
