# -*- coding: utf-8 -*-
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import stats
from scipy.stats import shapiro
from scipy.stats import wilcoxon
from pandas import Categorical
import itertools as it
from bisect import bisect_left
from typing import List
import scipy.stats as ss

def VD_A(treatment: List[float], control: List[float]):
    """
    Computes Vargha and Delaney A index
    A. Vargha and H. D. Delaney.
    A critique and improvement of the CL common language
    effect size statistics of McGraw and Wong.
    Journal of Educational and Behavioral Statistics, 25(2):101-132, 2000
    The formula to compute A has been transformed to minimize accuracy errors
    See: http://mtorchiano.wordpress.com/2014/05/19/effect-size-of-r-precision/
    :param treatment: a numeric list
    :param control: another numeric list
    :returns the value estimate and the magnitude
    """
    m = len(treatment)
    n = len(control)

    if m != n:
        raise ValueError("Data d and f must have the same length")

    r = ss.rankdata(treatment + control)
    r1 = sum(r[0:m])

    # Compute the measure
    # A = (r1/m - (m+1)/2)/n # formula (14) in Vargha and Delaney, 2000
    A = (2 * r1 - m * (m + 1)) / (2 * n * m)  # equivalent formula to avoid accuracy errors

    levels = [0.147, 0.33, 0.474]  # effect sizes from Hess and Kromrey, 2004
    magnitude = ["negligible", "small", "medium", "large"]
    scaled_A = (A - 0.5) * 2

    magnitude = magnitude[bisect_left(levels, abs(scaled_A))]
    estimate = A

    return estimate, magnitude

#Testing for normality, takes in a column of data and outputs whether its normal or not
def do_shapiro(column):
    stat, p = shapiro(column)
    print ("stat=%.3f, p=%.3f" % (stat, p))
    if p > 0.05:
        print ("Probably normal")
    else:
        print ("Probably not normal")


data = pd.read_csv("analysis.csv")
data.head()
col_avg_rt_t = data["Average_route_time_per_ac"]
col_total_taxi = data["Simulation_Time"]
col_max_rt_t = data["Max_route_time_per_ac"]
col_comp_time = data["Max_t_alg"]

print("\nTesting normality for average route time per ac")
col1 = col_avg_rt_t[0:100]
col2 = col_avg_rt_t[100:200]
col3 = col_avg_rt_t[200:300]
col4 = col_avg_rt_t[300:400]
col5 = col_avg_rt_t[400:500]
col6 = col_avg_rt_t[500:600]
do_shapiro(col1)
do_shapiro(col2)
do_shapiro(col3)
do_shapiro(col4)
do_shapiro(col5)
do_shapiro(col6)

print("\nTesting normality for total taxi time")
col1 = col_total_taxi[0:100]
col2 = col_total_taxi[100:200]
col3 = col_total_taxi[200:300]
col4 = col_total_taxi[300:400]
col5 = col_total_taxi[400:500]
col6 = col_total_taxi[500:600]


do_shapiro(col1)
do_shapiro(col2)
do_shapiro(col3)
do_shapiro(col4)
do_shapiro(col5)
do_shapiro(col6)

print("\nTesting normality for Max route time per ac")

col1 = col_max_rt_t[0:100]
col2 = col_max_rt_t[100:200]
col3 = col_max_rt_t[200:300]
col4 = col_max_rt_t[300:400]
col5 = col_max_rt_t[400:500]
col6 = col_max_rt_t[500:600]


do_shapiro(col1)
do_shapiro(col2)
do_shapiro(col3)
do_shapiro(col4)
do_shapiro(col5)
do_shapiro(col6)

print("\nTesting normality for max computation time")

col1 = col_comp_time[0:100]
col2 = col_comp_time[100:200]
col3 = col_comp_time[200:300]
col4 = col_comp_time[300:400]
col5 = col_comp_time[400:500]
col6 = col_comp_time[500:600]

do_shapiro(col1)
do_shapiro(col2)
do_shapiro(col3)
do_shapiro(col4)
do_shapiro(col5)
do_shapiro(col6)

#Plotting
#fig, axes = plt.subplots(1,1, figsize=(8,6))
#axes.hist(col_total_taxi[201:301])
#plt.ylabel('Frequency')
#plt.xlabel('Total taxi time')
#fig.tight_layout()
#plt.show()

#t_value,p_value=stats.ttest_rel(col_avg_rt_t[0:100],col_avg_rt_t[500:600])
#
#one_tailed_p_value=float("{:.6f}".format(p_value/2)) 
#
#print('Test statistic is %f'%float("{:.6f}".format(t_value)))
#
#print('p-value for one_tailed_test is %f'%one_tailed_p_value)
#
#alpha = 0.05
#
#if one_tailed_p_value<=alpha:
#
#    print('Conclusion','n','Since p-value(=%f)'%one_tailed_p_value,'<','alpha(=%.2f)'%alpha,'''We reject the null hypothesis H0. 
#
#So we conclude that the simulation has worsened by the difference in demand. i.e., d = 0 at %.2f level of significance.'''%alpha)
#
#else:
#
#    print('Conclusion','n','Since p-value(=%f)'%one_tailed_p_value,'>','alpha(=%.2f)'%alpha,'''We do not reject the null hypothesis H0. 
#
#So we conclude that the simulation results have not been affected by the difference in demand. i.e., d = 0 at %.2f level of significance.'''%alpha)

##Unpaired testing
#t_value,p_value=stats.ttest_ind(col_avg_rt_t[100:200],col_avg_rt_t[0:100])
#
#print('Test statistic is %f'%float("{:.6f}".format(t_value)))
#
#print('p-value for two tailed test is %f'%p_value)
#
#alpha = 0.05
#
#if p_value<=alpha:
#
#    print('Conclusion','n','Since p-value(=%f)'%p_value,'<','alpha(=%.2f)'%alpha,'''We reject the null hypothesis H0. So we conclude that the 
#
#effect of planner1 and planner2 on data output are not equal i.e., μ1 = μ2 at %.2f level of significance.'''%alpha)
#
#else:
#
#    print('Conclusion','n','Since p-value(=%f)'%p_value,'>','alpha(=%.2f)'%alpha,'''We do not reject the null hypothesis H0.
#effect of planner1 and planner2 on data output are equal i.e., μ1 = μ2 at %.2f level of significance.'''%alpha)
    
array1 = np.array(col_max_rt_t[100:200])
array2 = np.array(col_max_rt_t[400:500])
subtracted_array = np.subtract(array1, array2)
subtracted = list(subtracted_array)
w, p = wilcoxon(subtracted)
print (w,p)
#w, p = wilcoxon(subtracted, alternative='greater')
#print (w,p)

#planner comparison for MRT MAT
col1MRT = list(col_comp_time[0:100])
col2MRT = list(col_comp_time[200:300])
c = VD_A(col1MRT, col2MRT)
print (c)

    