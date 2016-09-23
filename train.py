import pandas as pd
import os 
import numpy as np
import scipy.stats as stats
from statsmodels.robust.scale import mad as mediad
from itertools import izip

def read_data():
	'''
	Read all the text files in the datasets folder
	concatenate them into one dataframe
	'''
	data_frames = []
	current_dir = os.getcwd()
	os.chdir('../datasets')
	for item in os.listdir(os.getcwd()):
		if os.path.isfile(item):
			if item.endswith('.txt'):
				data_frames.append(pd.read_csv(item, header=None, usecols=[0, 1, 2, 24]))
	os.chdir(current_dir)
	return pd.concat(data_frames)

def split_activities(data_frame):
	'''
	split accelerometer readings according to activities
	combine some classes 
	return three dataframes
	'''
	data_frame.columns = ['X', 'Y', 'Z', 'Activity']
	stationary_data = data_frame[data_frame['Activity'].isin([1, 2, 3])]
	walking_data = data_frame[data_frame['Activity'].isin([7, 8, 9])]
	running_data = data_frame[data_frame['Activity'] == 10]
	stationary_data['Activity'] = 1
	walking_data['Activity'] = 2
	running_data['Activity'] = 3	
	stationary_data = stationary_data.reset_index(drop=True)
	walking_data = walking_data.reset_index(drop=True)
	running_data = running_data.reset_index(drop=True)
	return stationary_data, walking_data, running_data

def get_parameters(data):
	activity = data['Activity']
	func_dict = {
		'min': np.min,
		'max': np.max,
		'diff': lambda x: np.max(x) - np.min(x),
		'mean': np.mean,
		'std': np.std,
		'iqr': stats.iqr,
		'rms': lambda x: np.sqrt(np.mean(np.square(x))),
		'integral': np.trapz,
		'mad': lambda x: x.mad(),
		'mediad': mediad
	}
	aggregations = {
		'X': func_dict,
		'Y': func_dict,
		'Z': func_dict
	}
	data_groups = data.groupby(data.index/200, as_index=False)
	stats_data = data_groups.agg(aggregations)
	correlations = data_groups[['X', 'Y', 'Z']].corr().unstack()
	correlations.columns = correlations.columns.droplevel()
	correlations.columns = ['XX', 'XY', 'XZ', 'YX', 'YY', 'YZ', 'ZX', 'ZY', 'ZZ']
	correlations = correlations.drop(['XX', 'YY', 'ZZ', 'YX', 'XZ', 'ZY'], axis=1)
	stats_data.columns = [''.join(col).strip() for col in stats_data.columns.values]
	stats_data = pd.concat([stats_data, correlations, activity[:len(stats_data)]], axis=1)
	return stats_data
		







