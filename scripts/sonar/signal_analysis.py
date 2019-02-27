import numpy as np


def fft(signal, fs=1e6, n=10240):
    """
    performs fft on a signal (uses blackman window)
    :param signal: input signal (1D array)
    :param fs: sampling frequency
    :param n: length of fft (if n > len(signal) zero padding is used)
    :return: peak frequency (Hz), peak frequency angle (deg), [fft_freq, fft_mag]
    """
    t = 1 / fs
    # remove dc component
    mean = np.mean(signal)
    signal = signal - mean
    # perform windowing and right hand fft
    window = np.blackman(len(signal))
    fft = np.fft.rfft(signal * window, n=n)
    # magnitude - normalize fft and multiply by 2 for one side
    fft_mag = 2 * np.abs(fft / n)
    # calculate fft frequencies
    fft_freq = np.fft.rfftfreq(n, t)
    # find max magnitude frequency and its phase angle
    peak_index = np.argmax(fft_mag)
    peak_freq = fft_freq[peak_index]
    peak_angle = np.angle(fft, deg=1)[peak_index]
    return peak_freq, peak_angle, [fft_freq, fft_mag]


def normalize_angle(angle, limit=180):
    """
    normalize angle between +/- limit
    :param angle: angle to normalize
    :param limit: limit (180 for degrees | PI for rad)
    :return: normalized angle
    """
    return (angle + limit) % (2 * limit) - limit