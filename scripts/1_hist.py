#!/usr/bin/env python3


"""
that’s all the Kalman filter is:
a Bayesian filter that assumes all the probability density functions involved in Gaussian distributions.
we are no longer modeling probabilities in a discrete state space, 
but will work with continuous variables.
a bar plot that shows the frequency of occurrences is called a Histogram

We could have aimed at building a more precise Bayes Filter by subdividing the corridor into smaller grid cells. If we choose very small grid cells, we will end up with a belief vector with several thousand
location probability values. Running a Bayes filter on huge data vectors at a rapid rate can produce a computational load that might be high enough to become impractical or even impossible to compute.
So, what do we do in order to increase the precision of our filter?
Let's take a second to recall the shape of the discrete distribution we used to model the noisy light sensor and noisy odometry sensor in the previous unit. Those distribuitons had one peak in the center
and spread symmetrically on both sides of the peak, resembling the shape of a bell.
How is this useful?
This is where Gaussian distributions come into play.




"""

import numpy as np
import scipy.stats
import matplotlib.pyplot as plt

mu = 10
sigma = 1

X = mu + sigma*np.random.randn(10000)

bins = 50

n, bins , patches = plt.hist(X, bins , density=1 , facecolor='green' , alpha=0.75 , edgecolor='black',linewidth=1.2)

y = scipy.stats.norm.pdf(bins , mu , sigma)
l = plt.plot(bins , y , 'r-' , linewidth=2 , label='Inline label')

ymax = y.max()

plt.xlabel('x variable')
plt.ylabel('Probability')
plt.title(r'$\mathrm{Histogram\:}\ \mu='+str(mu)+',\ \sigma='+str(sigma)+'$')
viewport = [mu-4*sigma, mu+4*sigma, 0 , ymax*1.1]
plt.axis(viewport)
plt.grid(True)
plt.show()