import math

def calcGaussian(x, mu, sigma2):
    gauss = math.exp(-(x-mu)**2 / (2*sigma2)) / math.sqrt(2*math.pi*sigma2)
    return gauss


if __name__ == "__main__":
    print( calcGaussian(8, 10, 4) )