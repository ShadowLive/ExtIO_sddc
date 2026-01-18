# ARM Optimization Testing Plan

## Current Test Coverage

### Correctness Tests (3 tests)
✅ **ConvertFloatCorrectness** - Verifies int16→float conversion
- Tests both randomization paths
- Compares NEON vs scalar output (bitwise identical for non-rand)
- **Coverage**: convert_float_neon()

✅ **ShiftFreqCorrectness** - Verifies complex multiplication
- 256 test cases with varied inputs
- Tolerance: 1e-5 (accounts for FP rounding differences)
- **Coverage**: shift_freq_neon()

✅ **CopyCorrectness** - Verifies complex copy/conjugate
- Tests both flip and non-flip modes
- **Coverage**: copy_neon()

### Functional Tests (45 tests)
✅ **Core streaming pipeline** - End-to-end data flow
✅ **FFT operations** - R2C and C2C transforms
✅ **Multiple decimation rates** - 0-4
✅ **Start/stop stress test** - Reliability
✅ **Data continuity** - No gaps over 15 seconds

### Performance Benchmarks
✅ **FFT benchmark** - Measures FFT backend performance
- R2C, C2C Forward, C2C Backward
- Multiple FFT sizes (128-4096)

## Testing Strategy for FMA + Compiler Flags

### Phase 1: Baseline Measurements (Before Changes)
```bash
# On mini.local
cd ~/ExtIO_sddc/build_m4

# 1. Run all tests to verify passing
./unittest/unittest 2>&1 | tee baseline_tests.txt

# 2. Capture baseline performance
./Core/fft_benchmark > baseline_fft.txt

# 3. Run NEON tests specifically
./unittest/unittest NeonOptimizations 2>&1 | tee baseline_neon.txt
```

### Phase 2: Apply Optimizations
1. Add FMA instructions to shift_freq_neon()
2. Add ARM compiler flags to CMakeLists.txt
3. Rebuild with new flags

### Phase 3: Verification (After Changes)
```bash
# 1. Verify all tests still pass
./unittest/unittest 2>&1 | tee optimized_tests.txt
diff baseline_tests.txt optimized_tests.txt

# 2. Verify NEON correctness (CRITICAL)
./unittest/unittest NeonOptimizations 2>&1 | tee optimized_neon.txt
# Should see: Passed: 3, Failed: 0

# 3. Measure performance improvement
./Core/fft_benchmark > optimized_fft.txt
```

### Phase 4: Performance Comparison
```bash
# Compare FFT times
echo "=== BASELINE ===" && cat baseline_fft.txt
echo "=== OPTIMIZED ===" && cat optimized_fft.txt
```

## Safety Checks

### 1. Numerical Accuracy
- ✅ Tolerance set to 1e-5 (sufficient for single-precision FP)
- ✅ FMA instructions maintain or improve numerical accuracy
- ✅ Tests compare NEON vs scalar reference

### 2. Regression Detection
- ✅ 48 functional tests ensure no behavior changes
- ✅ Correctness tests are deterministic
- ✅ Streaming tests verify real-world usage

### 3. Build Safety
- ✅ Compiler flags only affect ARM builds
- ✅ x86_64 build remains unchanged
- ✅ Fallback to scalar code if NEON unavailable

## Expected Results

### FMA Optimization
- **Performance**: 10-15% faster complex multiplication
- **Accuracy**: Same or better (FMA has better precision)
- **Tests**: Should pass with same tolerance (1e-5)

### Compiler Flags
- **Performance**: 5-10% overall improvement
- **Accuracy**: May have minor FP differences (within tolerance)
- **Tests**: All should pass

## Rollback Plan

If tests fail:
1. Revert changes: `git checkout HEAD -- Core/fft_mt_r2iq_neon.cpp Core/CMakeLists.txt`
2. Rebuild: `make clean && make -j8`
3. Verify baseline: `./unittest/unittest`

## Success Criteria

✅ All 48 tests pass
✅ NEON correctness tests pass (3/3)
✅ Measurable performance improvement (5%+)
✅ No numerical regressions within tolerance
