import numpy as np

# for more distributions see: https://numpy.org/doc/stable/reference/random/generator.html#distributions

_rng = np.random.default_rng()


def setSeed(seed):
    global _rng
    _rng = np.random.default_rng(seed)


def getSeededStatePreview():
    return str(_rng.bit_generator.state.get("state", {}))


def applyNoise(mu, sigma):
    return _rng.normal(mu, sigma, None)
