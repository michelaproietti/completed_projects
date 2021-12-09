import numpy as np
import os
import shutil
import glob
import csv
import numpy as np

# Generate scale and offset based on running mean and stddev along axis=0
class Scaler(object):
    """   
        offset = running mean
        scale = 1 / (stddev + 0.1) / 3 (i.e. 3x stddev = +/- 1.0)
    """
    def __init__(self, obs_dim): # obs_dim: dimension of axis=1 
        self.vars = np.zeros(obs_dim)
        self.means = np.zeros(obs_dim)
        self.m = 0
        self.n = 0
        self.first_pass = True

	# Update running mean and variance (this is an exact method)
    def update(self, x): # x: NumPy array, shape = (N, obs_dim)
        # given two different distributions with different means and variances,
        # we compute the combined mean and variance
        if self.first_pass:
            self.means = np.mean(x, axis=0)
            self.vars = np.var(x, axis=0)
            self.m = x.shape[0]
            self.first_pass = False
        else:
            n = x.shape[0]
            new_data_var = np.var(x, axis=0)
            new_data_mean = np.mean(x, axis=0)
            new_data_mean_sq = np.square(new_data_mean)
            new_means = ((self.means * self.m) + (new_data_mean * n)) / (self.m + n)
            self.vars = (((self.m * (self.vars + np.square(self.means))) +
                          (n * (new_data_var + new_data_mean_sq)))
                         / (self.m + n) - np.square(new_means))
            self.vars = np.maximum(0.0, self.vars)  # clip if it goes negative (only occasionally)
            self.means = new_means
            self.m += n

    def get(self): # returns 2-tuple: (scale, offset)
        return 1/(np.sqrt(self.vars) + 0.1)/3, self.means


class Logger(object): # Simple training logger: saves to file and optionally prints to stdout
    def __init__(self, logname, now): # now: unique sub-directory name (e.g. date/time string)
        path = os.path.join('log-files', logname, now)
        os.makedirs(path)
        filenames = glob.glob('*.py')  # put copy of all python files in log_dir
        for filename in filenames:     # for reference
            shutil.copy(filename, path)
        path = os.path.join(path, 'log.csv')
        self.write_header = True
        self.log_entry = {}
        self.f = open(path, 'w')
        self.writer = None  # DictWriter created with first call to write() method

    def write(self, display=True): # Write 1 log entry to file, and optionally to stdout (Log fields preceded by '_' will not be printed to stdout)
        if display:
            self.disp(self.log_entry)
        if self.write_header:
            fieldnames = [x for x in self.log_entry.keys()]
            self.writer = csv.DictWriter(self.f, fieldnames=fieldnames)
            self.writer.writeheader()
            self.write_header = False
        self.writer.writerow(self.log_entry)
        self.log_entry = {}
    
    # Print metrics to stdout
    @staticmethod
    def disp(log):
        log_keys = [k for k in log.keys()]
        log_keys.sort()
        print('***** Episode {}, Mean R = {:.1f} *****'.format(log['_Episode'], log['_MeanReward']))
        for key in log_keys:
			# don't display log items that starts with '_'
            if key[0] != '_': print('{:s}: {:.3g}'.format(key, log[key]))
        print('\n')
    
    # Update fields in log (does not write to file, used to collect updates.
    def log(self, items): # items is a dictionary of items to update
        self.log_entry.update(items)

    def close(self): # Close log file - WARNING: log cannot be written after calling this
        self.f.close()
