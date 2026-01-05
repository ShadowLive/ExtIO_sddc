
{
	const int decimate = this->mdecimation;
	const int mfft = this->mfftdim[decimate];	// = halfFft / 2^mdecimation
	const fftwf_complex* filter = filterHw[decimate];
	const bool lsb = this->getSideband();
	const auto filter2 = &filter[halfFft - mfft / 2];

	// Use local variable instead of class member to avoid race condition
	fftwf_plan local_plan_f2t_c2c = plans_f2t_c2c[decimate];

	// Decimation mask for calculating position within output block
	const int decimateMask = (1 << decimate) - 1;

	while (r2iqOn) {
		const int16_t *dataADC;
		const int16_t *endloop;
		uint64_t mySeq;

		const int _mtunebin = this->mtunebin;

		// === INPUT SECTION (serialized) ===
		{
			std::unique_lock<std::mutex> lk(mutexR2iqControl);
			dataADC = inputbuffer->getReadPtr();

			if (!r2iqOn)
				return 0;

			mySeq = inputSeq.fetch_add(1);
			this->bufIdx = (this->bufIdx + 1) % QUEUE_SIZE;
			endloop = inputbuffer->peekReadPtr(-1) + transferSamples - halfFft;
		}

		auto inloop = th->ADCinTime;

#if PRINT_INPUT_RANGE
		std::pair<int16_t, int16_t> blockMinMax = std::make_pair<int16_t, int16_t>(0, 0);
#endif
		if (!this->getRand())
		{
			convert_float<false>(endloop, inloop, halfFft);
#if PRINT_INPUT_RANGE
			auto minmax = std::minmax_element(dataADC, dataADC + transferSamples);
			blockMinMax.first = *minmax.first;
			blockMinMax.second = *minmax.second;
#endif
			convert_float<false>(dataADC, inloop + halfFft, transferSamples);
		}
		else
		{
			convert_float<true>(endloop, inloop, halfFft);
			convert_float<true>(dataADC, inloop + halfFft, transferSamples);
		}

#if PRINT_INPUT_RANGE
		th->MinValue = std::min(blockMinMax.first, th->MinValue);
		th->MaxValue = std::max(blockMinMax.second, th->MaxValue);
		++th->MinMaxBlockCount;
		if (th->MinMaxBlockCount * processor_count / 3 >= DEFAULT_TRANSFERS_PER_SEC)
		{
			float minBits = (th->MinValue < 0) ? (log10f((float)(-th->MinValue)) / log10f(2.0f)) : -1.0f;
			float maxBits = (th->MaxValue > 0) ? (log10f((float)(th->MaxValue)) / log10f(2.0f)) : -1.0f;
			printf("r2iq: min = %d (%.1f bits) %.2f%%, max = %d (%.1f bits) %.2f%%\n",
				(int)th->MinValue, minBits, th->MinValue * -100.0f / 32768.0f,
				(int)th->MaxValue, maxBits, th->MaxValue * 100.0f / 32768.0f);
			th->MinValue = 0;
			th->MaxValue = 0;
			th->MinMaxBlockCount = 0;
		}
#endif
		dataADC = nullptr;
		inputbuffer->ReadDone();

		// Calculate parameters for frequency shift
		const auto count = std::min(mfft / 2, halfFft - _mtunebin);
		const auto source = &th->ADCinFreq[_mtunebin];
		const auto start = std::max(0, mfft / 2 - _mtunebin);
		const auto source2 = &th->ADCinFreq[_mtunebin - mfft / 2];
		const auto dest = &th->inFreqTmp[mfft / 2];

		// === FFT + OUTPUT with per-k synchronization ===
		// Each k iteration: do FFT (parallel), then copy to output (serialized)
		for (int k = 0; k < fftPerBuf; k++)
		{
			// FFT processing - can run in parallel across threads
			// (each thread uses its own th->ADCinTime, th->ADCinFreq, th->inFreqTmp)
			fftwf_execute_dft_r2c(plan_t2f_r2c, th->ADCinTime + (3 * halfFft / 2) * k, th->ADCinFreq);

			shift_freq(th->inFreqTmp, source, filter, 0, count);
			if (mfft / 2 != count)
				memset(th->inFreqTmp[count], 0, sizeof(float) * 2 * (mfft / 2 - count));

			shift_freq(dest, source2, filter2, start, mfft / 2);
			if (start != 0)
				memset(th->inFreqTmp[mfft / 2], 0, sizeof(float) * 2 * start);

			fftwf_execute_dft(local_plan_f2t_c2c, th->inFreqTmp, th->inFreqTmp);

			// === OUTPUT COPY (serialized in sequence order) ===
			{
				std::unique_lock<std::mutex> lk(outputMutex);

				// Wait for our turn (only on first k of this block)
				if (k == 0) {
					outputCV.wait(lk, [&] { return outputWriteTurn.load() == mySeq || !r2iqOn; });
					if (!r2iqOn)
						return 0;

					// Get output pointer
					int myDecimateCount = static_cast<int>(mySeq & decimateMask);
					if (myDecimateCount == 0) {
						sharedPout = (fftwf_complex*)outputbuffer->getWritePtr();
					}
				}

				// Calculate output position
				int myDecimateCount = static_cast<int>(mySeq & decimateMask);
				fftwf_complex* pout = sharedPout;
				for (int i = 0; i < myDecimateCount; i++) {
					pout += mfft / 2 + (3 * mfft / 4) * (fftPerBuf - 1);
				}

				// Copy to output
				if (lsb)
				{
					if (k == 0)
						copy<true>(pout, &th->inFreqTmp[mfft / 4], mfft / 2);
					else
						copy<true>(pout + mfft / 2 + (3 * mfft / 4) * (k - 1), &th->inFreqTmp[0], (3 * mfft / 4));
				}
				else
				{
					if (k == 0)
						copy<false>(pout, &th->inFreqTmp[mfft / 4], mfft / 2);
					else
						copy<false>(pout + mfft / 2 + (3 * mfft / 4) * (k - 1), &th->inFreqTmp[0], (3 * mfft / 4));
				}

				// On last k, complete the block
				if (k == fftPerBuf - 1) {
					if (myDecimateCount == decimateMask) {
						outputbuffer->WriteDone();
						sharedPout = nullptr;
					}
					outputWriteTurn.fetch_add(1);
					outputCV.notify_all();
				}
			}
		}

	} // while(run)
	return 0;
}
