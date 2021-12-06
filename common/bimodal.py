'''
Test bimodality of the velocities and distances in the experiments, calculate parameters and plot bimodal distroibution over the data.
'''
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt
from collections import Counter, OrderedDict
from math import sqrt
import bisect

def otsu(histogramCounts, bins=[]):
  '''
  Calculate the threshold by Otsu's method. Adapted from Wikipedia's article.
  Argument:
    histogramCounts: either an OrderedDict object containing the number of occurrences for each value or the counts. If the second argument is empty, it is the OrderedDict. The dict must be ordered by the key.
    bins: a array containing the bins in the histogram. If it is not empty, x is the counts in each bin. bins has the size of x plus one.
  Return:
    cut: value that cut the histogram in two normal curves;
    mean0, mean1: mean of the normal curve on the left and right sides of the cut value, respectively;
    w0, w1: weight of the left and right sides normal curves, respectively, such that w0 + w1 = 1.
  '''
  if len(bins) == 0:
    counts = tuple(histogramCounts.values())
    values = tuple(histogramCounts.keys())
  else:
    values = bins[:-1]
    counts = histogramCounts
  total = sum(counts);
  top = max(values);
  sumB = 0.;
  wB = 0;
  maximum = 0.0;
  sum1 = np.dot(counts,values)
  for i in range(len(values)):
    wF = total - wB
    if wB > 0 and wF > 0:
      mF = (sum1 - sumB) / wF;
      val = wB * wF * ((sumB / wB) - mF) * ((sumB / wB) - mF);
      if val >= maximum:
        cut = values[i];
        mean0 = sumB / wB
        mean1 = mF
        w0 = wB
        w1 = wF
        maximum = val;
    wB = wB + counts[i];
    sumB = sumB + values[i] * counts[i];
  return cut, mean0, mean1, w0/total, w1/total

def isNormal(x,alpha=1e-3):
  '''
  Returns True if normal distribution test returns a p-value at least alpha.
  '''
  k2, p = stats.normaltest(x)
  # null hypothesis (when p < alpha): x comes from a normal distribution
  return p >= alpha
  
def print_normal_test_result(x,varname,alpha=1e-3):
  '''
  Prints if normal distribution test returns a p-value at least alpha.
  '''
  isnormal = isNormal(x,alpha)
  if isnormal:  
    print(varname + " comes from a normal distribution.")
  else:
    print(varname + " does not come from a normal distribution.")
  return isnormal

def bimodal_normal_parameters(x, bins = []):
  '''
  Assuming the data comes from a bimodal normal distribution, it calculates the parameters.
  Argument: 
    x: either the data or the counts. If the second argument is empty, it is the data.
    bins: a array containing the bins in the histogram. If it is not empty, x is the counts in each bin. bins has the size of x plus one.
  Return:
    mean0, mean1: mean of the normal curve on the left and right sides, respectively;
    w0, w1: weight of the left and right sides normal curves, respectively, such that w0 + w1 = 1;
    std1, std2: standard deviation of the normal curve on the left and right sides, respectively.
  '''
  if len(bins) == 0:
    cut, mean1, mean2, w1, w2 = otsu(OrderedDict(sorted(Counter(x).items())))
    l1 = [i for i in x if i <= cut]
    l2 = [i for i in x if i > cut]
    std1 = np.std(l1)
    std2 = np.std(l2)
  else:
    cut, mean1, mean2, w1, w2 = otsu(x,bins)
    i = bisect.bisect_right(bins[:-1],cut)
    total = sum(x)
    std1 = sqrt(sum([x[j]*(bins[j] - mean1)**2 for j in range(i)])/(w1*total))
    std2 = sqrt(sum([x[j]*(bins[j] - mean2)**2 for j in range(i,len(bins)-1)])/(w2*total))
  return mean1, mean2, w1, w2, std1, std2

def plotHistogramAndEstimatedBimodal(counts,bins,numBins = 100):
  mean1, mean2, w1, w2, std1, std2 = bimodal_normal_parameters(counts,bins)
  v = np.linspace(min(bins), max(bins), numBins)
  plt.plot(v, w1*stats.norm.pdf(v, mean1, std1)+w2*stats.norm.pdf(v, mean2, std2))
  plt.hist(bins[:-1], bins, weights=counts,density=True)

def test_data():
  rng = np.random.default_rng()
  pts = 1000
  mu1,sigma1,mu2,sigma2 = 2.5,0.5,1,1
  a = rng.normal(mu1, sigma1, size=pts)
  b = rng.normal(mu2, sigma2, size=pts)
  x = np.concatenate((a, b))
  print_normal_test_result(x,'x')
  print_normal_test_result(a,'a')
  print_normal_test_result(b,'b')
  mean1, mean2, w1, w2, std1, std2, cut = bimodal_normal_parameters(x)
  print(mean1, mean2, w1, w2, std1, std2, cut)
  plt.axvline(cut,color='black')
  v = np.linspace(min(x), max(x), 100)
  plt.plot(v, 0.5*stats.norm.pdf(v, mu1, sigma1)+0.5*stats.norm.pdf(v, mu2, sigma2),color='red')
  plt.plot(v, w1*stats.norm.pdf(v, mean1, std1)+w2*stats.norm.pdf(v, mean2, std2),color='green')
  plt.hist(x,bins=100,density=True)
  plt.show()

def test_hist():
  numBins = 100
  rng = np.random.default_rng()
  pts = 1000
  mu1,sigma1,mu2,sigma2 =  2.5,0.5,1,1
  a = rng.normal(mu1, sigma1, size=pts)
  b = rng.normal(mu2, sigma2, size=pts)
  x = np.concatenate((a, b))
  counts, bins = np.histogram(x,bins=numBins)
  mean1, mean2, w1, w2, std1, std2, cut = bimodal_normal_parameters(counts,bins)
  print(mean1, mean2, w1, w2, std1, std2, cut)
  plt.axvline(cut,color='black')
  v = np.linspace(min(bins), max(bins), numBins)
  plt.plot(v, 0.5*stats.norm.pdf(v, mu1, sigma1)+0.5*stats.norm.pdf(v, mu2, sigma2),color='red')
  plt.plot(v, w1*stats.norm.pdf(v, mean1, std1)+w2*stats.norm.pdf(v, mean2, std2),color='green')
  plt.hist(x,bins=numBins,density=True)
  plt.show()


