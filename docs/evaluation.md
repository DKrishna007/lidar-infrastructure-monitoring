# Evaluation Metrics

## Before vs After System Deployment

| Metric | Before | After | Improvement |
|--------|--------|-------|-------------|
| Defect detection accuracy | 60–65% | 80–82% | **+20pp** |
| False positive rate | ~35% | ~10% | **-25pp** |
| Inspection time per structure | 10–14 hrs | 2–4 hrs | **-8–12 hrs** |
| Cost per inspection cycle | Baseline | 40–55% lower | **-40–55%** |

## ICP Registration Quality

| Structure | Fitness Score | RMSE (m) | Status |
|-----------|-------------|----------|--------|
| bridge_span_A | 0.982 | 0.00041 | ✅ |
| bridge_span_B | 0.971 | 0.00089 | ✅ |
| pier_east | 0.963 | 0.00112 | ✅ |
| column_01 | 0.988 | 0.00028 | ✅ |

## Defect Threshold Analysis

| Threshold | Detected | False Positives | Recommendation |
|-----------|----------|----------------|----------------|
| 1cm | High | Very High | Too sensitive |
| **3cm** | **80–82%** | **~10%** | **✅ Optimal** |
| 5cm | ~70% | ~5% | Misses small defects |
