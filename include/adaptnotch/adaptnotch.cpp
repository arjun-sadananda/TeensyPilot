/**
 * @file adaptnotch.cpp
 * @brief Adaptive notch filtering algorithm
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 8 Dec 2020
 */

#include "adaptnotch.h"

namespace adaptnotch {

constexpr double pi = 3.14159265358979323846;

AdaptiveNotch::AdaptiveNotch(const Params& params)
  : params_(params), buffer_(params.NFFT)
{
  fft_bin_count_ = params_.NFFT / 2;

  // two notch filters can be composed into a dual notch filter to increase
  // the stopband without increasing sample delay too much.
  dual_notch_ = (params_.dual_notch_width_percent != 0);
  if (dual_notch_) {
    notch1_ctr_ = 1 - params_.dual_notch_width_percent / 100.0;
    notch2_ctr_ = 1 + params_.dual_notch_width_percent / 100.0;
  }

  Q_ = params_.Q / 100.0;

  // calculate frequency range to search for peaks over
  min_hz_ = params_.min_hz;
  max_hz_ = std::max(2 * min_hz_, params_.max_hz);

  // compute the maximum sample rate required to analyze the spectrum with a
  // max frequency component at the max peak searching range. We will use this
  // frequency to downsample the input data for a more efficient FFT.
  const double nyquist = 2. * max_hz_;
  max_samples_ = std::max(1, static_cast<int>(params_.Fs / nyquist));
  fftFs_ = params_.Fs / max_samples_;

  fres_ = fftFs_ / static_cast<double>(params_.NFFT);
  start_bin_ = std::max(params_.start_bin, static_cast<int>(min_hz_ / fres_));

  // construct a window for performing FFT on real-time data
  window_ = windowHann(params_.NFFT);

  fft_.SetFlag(Eigen::FFT<double>::Unscaled);
  fft_.SetFlag(Eigen::FFT<double>::HalfSpectrum);

  // initialize state variables
  reset();
}

// ----------------------------------------------------------------------------

double AdaptiveNotch::apply(double x)
{
  // accumulate input samples to downsample and respect FFT rate
  input_accumulator_ += x;
  input_samples_++;

  if (input_samples_ == max_samples_) {
    // downsample to FFT rate
    const double sample = input_accumulator_ / max_samples_;

    // reset accumulator
    input_accumulator_ = 0;
    input_samples_ = 0;

    buffer_.add(sample);

    peakFreq_ = findFreqPeak();

    // update notch filters
    if (dual_notch_) {
      notch1_->update(peakFreq_ * notch1_ctr_, params_.Fs, Q_);
      notch2_->update(peakFreq_ * notch2_ctr_, params_.Fs, Q_);
    } else {
      notch1_->update(peakFreq_, params_.Fs, Q_);
    }
  }

  //
  // Apply notch filter
  //

  double y = notch1_->apply(x);
  if (dual_notch_) y = notch2_->apply(y);

  return y;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void AdaptiveNotch::reset()
{
  // initialize peak frequency estimator
  peakFreq_ = max_hz_;

  // reset downsample accumulator
  input_accumulator_ = 0;
  input_samples_ = 0;

  // initialize notch filters
  if (dual_notch_) {
    notch1_.reset(new BiquadNotch(peakFreq_ * notch1_ctr_, params_.Fs, Q_));
    notch2_.reset(new BiquadNotch(peakFreq_ * notch2_ctr_, params_.Fs, Q_));
  } else {
    notch1_.reset(new BiquadNotch(peakFreq_, params_.Fs, Q_));
  }
}

// ----------------------------------------------------------------------------

Eigen::VectorXd AdaptiveNotch::windowHann(int N)
{
  Eigen::VectorXd w = Eigen::VectorXd::Zero(N);
  for (size_t i=0; i<N; ++i) {
    w(i) = 0.5 - 0.5 * std::cos( 2 * pi * i / (N-1) );
  }
  return w;
}

// ----------------------------------------------------------------------------

double AdaptiveNotch::findFreqPeak()
{
  // compute DFT of buffered window of samples
  Eigen::VectorXcd Yc;
  const Eigen::VectorXd y = buffer_.sequentialView().cwiseProduct(window_);
  fft_.fwd(Yc, y);

  // extract real part
  Y_ = Yc.array().abs();

  //
  // Find max bin and max / min components
  //

  double dataMax = 0;
  double dataMin = 1, dataMinHi = 1;
  int binMax = 0;

  // max bin/component
  for (size_t i=start_bin_; i<fft_bin_count_; i++) {
    if (Y_(i) > dataMax) {
      dataMax = Y_(i);
      binMax = i;
    }
  }

  if (binMax == 0) {
    // edge case, don't do anything
    binMax = static_cast<int>(peakFreq_ / fres_);
  } else {
    // look for the min below the max peak
    for (size_t i=binMax-1; i>1; i--) {
      dataMin = Y_(i);
      // break if the bin below will increase
      if (Y_(i-1) > Y_(i)) break;
    }
    // look for the min above the max peak
    for (size_t i=binMax+1; i<fft_bin_count_-1; i++) {
      dataMinHi = Y_(i);
      // break if the bin above will increase
      if (Y_(i) < Y_(i+1)) break;
    }
    dataMin = std::min(dataMin, dataMinHi);
  }

  //
  // Recover peak frequency
  //

  // weighted mean using peak and shoulder bins
  double sq = Y_(binMax) * Y_(binMax);
  double fftSum = sq;
  double fftWeightedSum = sq * binMax;

  // accumulate upper shoulder unless it would be Nyquist bin
  int shoulderBin = binMax + 1;
  if (shoulderBin < fft_bin_count_) {
    sq = Y_(shoulderBin) * Y_(shoulderBin);
    fftSum += sq;
    fftWeightedSum += sq * shoulderBin;
  }

  // accumulate lower shoulder unless it would be DC bin
  if (binMax > 1) {
    shoulderBin = binMax - 1;
    sq = Y_(shoulderBin) * Y_(shoulderBin);
    fftSum += sq;
    fftWeightedSum += sq * shoulderBin;
  }

  // calculate peak freq from weighted bins
  double centerFreq = peakFreq_;
  if (fftSum > 0) {
    const double meanIdx = fftWeightedSum / fftSum;
    centerFreq = meanIdx * fres_;
  }
  centerFreq = utils::clamp(centerFreq,
                static_cast<double>(min_hz_), static_cast<double>(max_hz_));

  //
  // Dynamic smoothing of peak freq estimate
  //

  // move to big peaks fast
  const double alpha = 0.1;
  const double gamma = utils::clamp(dataMax / dataMin, 1., 0.8/alpha);
  centerFreq = peakFreq_ + alpha * gamma * (centerFreq - peakFreq_);

  return centerFreq;
}

} // ns adaptnotch
