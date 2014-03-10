import numpy as np

from librosa.feature import melspectrogram
from nmf_kl import KLdivNMF as NMF


def spectro(sr, data):
    hop = int(.1 * sr)  # 1s
    S = melspectrogram(data, sr=sr, n_fft=2048, hop_length=hop,
                                       n_mels=128, fmax=1500)
    return S.T


class Perception(object):
    """Implements nmf decomposition and best activation.
    """

    def _init_nmf(self, n_components):
        self.nmf = NMF(n_components)

    def train(self, n_components, data):
        self._init_nmf(n_components)
        self.nmf.fit_transform(data)

    def load(self, dico):
        self._init_nmf(dico.shape[0])
        self.nmf.components_ = dico

    def perceive(self, data):
        acti = self.nmf.transform(data)
        best = np.argmax(acti, axis=1)
        counts = np.zeros((self.nmf.n_components,))
        for i in best:
            counts[i] += 1
        return np.argmax(counts)

    @classmethod
    def load_fromfile(cls, f):
        a=np.load(f)
        p=cls()
        p.load(a['dico'])

        return p
