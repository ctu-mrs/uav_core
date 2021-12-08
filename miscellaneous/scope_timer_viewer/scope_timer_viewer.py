#!/usr/bin/python3

import rosbag
import os
import sys
import numpy as np

import matplotlib.pyplot as plt
import matplotlib as mpl

# # #{ print_help
def print_help():
    print('From scope timer logs (.log/.csv/.txt; format: [scope,label_from,label_to,sec_start,sec_end,sec_duration]), print out and plot the timers.')
    print('Usage:')
    print('A) ./scope_timer_viewer.py scope_timer_log.log [..]          -> process a single file')
    print('B) ./scope_timer_viewer.py logs_directory [..]               -> process all files within the given directory')
    print('C) ./scope_timer_viewer.py first_log.log second_log.log [..] -> process both logs and compare them')
    print('D) ./scope_timer_viewer.py first_dir second_dir [..]         -> process both directories and compare all logs')
    print('Optional parameters:')
    print('--scope scope_name (string, default: "" (all scopes)) -> only scope with name \"scope_name\" will be processed')
    print('--checkpoints (bool flag)                             -> all checkpoints within scopes will be processed')
    print('--plot (bool flag)                                    -> data will be plotted')
# # #}

# # #{ load_params
def load_params(args):
    path_A = args[1]

    if path_A in ('-h', '--h', '--help', 'help', '-help'):
        print_help()
        exit(0)

    path_A      = os.path.abspath(path_A)
    path_B      = None
    scope       = None
    checkpoints = False
    plot        = False

    # It's ugly but it works somehow (am slightly drunk, woke up at 4AM, and am writing this in the plane without internet)
    argc = len(args)
    if argc > 2:
        
        start_idx = 2
        if not args[2].startswith('--'):
            path_B = os.path.abspath(args[2])
            start_idx = 3
        
        for i in range(start_idx, argc):
            if args[i] == '--scope' and i + 1 < argc:
                scope = str(args[i+1])
            elif args[i] == '--checkpoints':
                checkpoints = True
            elif args[i] == '--plot':
                plot = True

    return path_A, path_B, scope, checkpoints, plot
# # #}

# # #{ DataLoader class

class DataLoader:

    SCOPE_START=''
    SCOPE_END=''
    SEC_FROM='sec_from'
    SEC_TO='sec_to'
    SEC_DURATION='sec_duration'
    SEC_DURATION_MEAN='sec_duration_mean'
    SEC_DURATION_STD='sec_duration_std'
    DATA='data'
    SUPPORTED_FORMATS=('.log', '.txt', '.csv')

    data             = (None, None)
    loading_from_dir = False

# # #{ load_file
    def load_file(self, filepath):
        # print('[load_file] loading file: {:s}'.format(filepath))

        data = {}

        with open(filepath, "r") as f:

            lines = f.readlines()
            for line in lines:

                if line.startswith("#"):
                    continue

                elements = line.split(',')
                if len(elements) != 6:
                    continue

                scope      = str(elements[0])
                label_from = str(elements[1])
                label_to   = str(elements[2])

                # Initialize triplet dictionaries
                if scope not in data.keys():
                    data[scope] = {}
                if label_from not in data[scope].keys():
                    data[scope][label_from] = {}
                if label_to not in data[scope][label_from].keys():
                    data[scope][label_from][label_to] = {}
                    data[scope][label_from][label_to][self.DATA] = []

                data[scope][label_from][label_to][self.DATA].append({self.SEC_FROM : float(elements[3]), self.SEC_TO : float(elements[4]), self.SEC_DURATION : float(elements[5])})

        if len(data.keys()) == 0:
            return None

        return data
# # #}

# # #{ load_dir
    def load_dir(self, dirpath):
        self.loading_from_dir = True
        data = {}

        for filename in os.listdir(dirpath):

            if not any([filename.endswith(fmt) for fmt in self.SUPPORTED_FORMATS]):
                continue

            filepath = os.path.join(dirpath, filename)
            filedata = self.load_file(filepath)
            if filedata is None:
                continue

            # Merge dicts (careful, filedata values override data values if a key is present in both dicts)
            # data = {**data, **filedata}
            
            # Merge dicts
            for key_scope in filedata:

                if not key_scope in data:
                    data[key_scope] = filedata[key_scope]
                    continue

                for key_label_from in filedata[key_scope]:

                    if not key_label_from in data[key_scope]:
                        data[key_scope][key_label_from] = filedata[key_scope][key_label_from]
                        continue

                    for key_label_to in filedata[key_scope][key_label_from]:

                        if not key_label_to in data[key_scope][key_label_from]:
                            data[key_scope][key_label_from][key_label_to]            = {}
                            data[key_scope][key_label_from][key_label_to][self.DATA] = filedata[key_scope][key_label_from][key_label_to][self.DATA]
                        else:
                            data[key_scope][key_label_from][key_label_to][self.DATA].extend(filedata[key_scope][key_label_from][key_label_to][self.DATA])

        if len(data.keys()) == 0:
            return None

        return data
# # #}

# # #{ load_files
    def load_files(self, filepath_A, filepath_B):
        filedata_A = self.load_file(filepath_A)
        filedata_B = self.load_file(filepath_B)

        if filedata_A is None or filedata_B is None:
            print('[load_files] failed to load any data from the files')
            return (None, None)

        return (filedata_A, filedata_B)
# # #}

# # #{ load_dirs
    def load_dirs(self, dirpath_A, dirpath_B):
        self.loading_from_dir = True

        dirdata_A = self.load_dir(dirpath_A)
        dirdata_B = self.load_dir(dirpath_B)

        if dirdata_A is None or dirdata_B is None:
            print('[load_dirs] failed to load any data from the directories')
            return (None, None)

        return (dirdata_A, dirdata_B)
# # #}

# # #{ load_data
    def load_data(self, path_A, path_B):
        is_dir_A = os.path.isdir(path_A)

        # Single file
        if path_B is None:

            if is_dir_A:
                self.data = (self.load_dir(path_A), None)
            else:
                self.data = (self.load_file(path_A), None)

        # Two files to compare
        else:

            is_dir_B = os.path.isdir(path_B)
            if is_dir_A and is_dir_B:
                self.data = self.load_dirs(path_A, path_B)
            elif not is_dir_A and not is_dir_B:
                self.data = self.load_files(path_A, path_B)
            else:
                print('[load_data] to compare two files/directories, two files/directories should be given as arguments')
                self.data = (None, None)
# # #}

# # #{ valid
    def valid(self):
        return self.data[0] is not None
# # #}

# # #}

# # #{ filter_data
def filter_data(data_loader, scope=None, checkpoints=False):

    filt_data = [None, None]

    for data_idx in (0, 1):

        if not data_loader.data[data_idx]:
            continue

        sparse_data = {}

        for key_scope in data_loader.data[data_idx]:

            # Skip other scopes
            if scope and key_scope != scope:
                continue

            # For each scope/checkpoint start
            for key_label_from in data_loader.data[data_idx][key_scope]:

                # Skip non-scope-starts if checkpoints are disabled
                if not checkpoints and key_label_from != data_loader.SCOPE_START:
                    continue

                # For each scope/checkpoint end
                for key_label_to in data_loader.data[data_idx][key_scope][key_label_from]:
                
                    # Skip non-scope-ends if checkpoints are disabled
                    if not checkpoints and key_label_to != data_loader.SCOPE_END:
                        continue

                    # Initialize triplet dictionaries
                    if key_scope not in sparse_data.keys():
                        sparse_data[key_scope] = {}
                    if key_label_from not in sparse_data[key_scope].keys():
                        sparse_data[key_scope][key_label_from] = {}
                    
                    # Copy data
                    sparse_data[key_scope][key_label_from][key_label_to] = data_loader.data[data_idx][key_scope][key_label_from][key_label_to]

        filt_data[data_idx] = sparse_data

    return tuple(filt_data)

# # #}

# # #{ process_data
def process_data(data_loader, scope=None, checkpoints=False):

    data = filter_data(data_loader, scope, checkpoints)
        

    for data_idx in (0, 1):

        if not data[data_idx]:
            continue

        for key_scope in data[data_idx]:
            for key_label_from in data[data_idx][key_scope]:
                for key_label_to in data[data_idx][key_scope][key_label_from]:

                    # Get statistics of durations
                    durations      = [element[data_loader.SEC_DURATION] for element in data[data_idx][key_scope][key_label_from][key_label_to][data_loader.DATA]]
                    durations_mean = np.mean(durations)
                    durations_std  = np.std(durations)

                    data[data_idx][key_scope][key_label_from][key_label_to][data_loader.SEC_DURATION_MEAN] = durations_mean
                    data[data_idx][key_scope][key_label_from][key_label_to][data_loader.SEC_DURATION_STD]  = durations_std

    data_loader.data = data
                    
# # #}

# # #{ print_data
def print_data(data_loader, path_A, path_B):

    if data_loader.data[1] is None:

        print('Statistics of logs from: {:s}'.format(path_A))
        print('----------------------------------------------------------')
        print('Scope | From | To | Mean duration (ms) | Std duration (ms)')

        for key_scope in data_loader.data[0]:
            for key_label_from in data_loader.data[0][key_scope]:
                for key_label_to in data_loader.data[0][key_scope][key_label_from]:
                    duration_mean_ms = data_loader.data[0][key_scope][key_label_from][key_label_to][data_loader.SEC_DURATION_MEAN] * 1000.0
                    duration_std_ms  = data_loader.data[0][key_scope][key_label_from][key_label_to][data_loader.SEC_DURATION_STD] * 1000.0
                    print('{:s} | {:s} | {:s} | {:.1f} | {:.1f}'.format(key_scope, key_label_from, key_label_to, duration_mean_ms, duration_std_ms))

    else:

        print('Statistics of logs from:')
        print('A: {:s}'.format(path_A))
        print('B: {:s}'.format(path_B))
        print('----------------------------------------------------------')
        for key_scope in data_loader.data[0]:
            if key_scope not in data_loader.data[1]:
                continue

            for key_label_from in data_loader.data[0][key_scope]:
                if key_label_from not in data_loader.data[1][key_scope]:
                    continue

                for key_label_to in data_loader.data[0][key_scope][key_label_from]:
                    if key_label_to not in data_loader.data[1][key_scope][key_label_from]:
                        continue

                    A_duration_mean_ms = data_loader.data[0][key_scope][key_label_from][key_label_to][data_loader.SEC_DURATION_MEAN] * 1000.0
                    A_duration_std_ms  = data_loader.data[0][key_scope][key_label_from][key_label_to][data_loader.SEC_DURATION_STD] * 1000.0
                    B_duration_mean_ms = data_loader.data[1][key_scope][key_label_from][key_label_to][data_loader.SEC_DURATION_MEAN] * 1000.0
                    B_duration_std_ms  = data_loader.data[1][key_scope][key_label_from][key_label_to][data_loader.SEC_DURATION_STD] * 1000.0

                    mean_ms_diff = (B_duration_mean_ms / A_duration_mean_ms) * 100.0
                    std_ms_diff = (B_duration_std_ms / A_duration_std_ms) * 100.0

                    print('{:s} | from: {:s} | to: {:s}'.format(key_scope, key_label_from, key_label_to))
                    print('   A -> mean: {:.1f} ms | mean: {:.1f}% | std: {:.1f} ms | std: {:.1f}%'.format(A_duration_mean_ms, 100.0, A_duration_std_ms, 100.0))
                    print('   B -> mean: {:.1f} ms | mean: {:.1f}% | std: {:.1f} ms | std: {:.1f}%'.format(B_duration_mean_ms, mean_ms_diff, B_duration_std_ms, std_ms_diff))
                    
# # #}

# # #{ plot_data
def plot_data(data_loader, path_A, path_B):

    # if data_loader.loading_from_dir:
    #     print('[plot_data] Entire directories will not be plotted (may contain files with same scopes but varying timestamps). Rerun for single file or file-to-file comparison.')
    #     return
    
    if data_loader.data[1] is None:

        for key_scope in data_loader.data[0]:
            for key_label_from in data_loader.data[0][key_scope]:
                for key_label_to in data_loader.data[0][key_scope][key_label_from]:
                    secs_from = [element[data_loader.SEC_FROM] for element in data_loader.data[0][key_scope][key_label_from][key_label_to][data_loader.DATA]]
                    secs_to   = [element[data_loader.SEC_TO] for element in data_loader.data[0][key_scope][key_label_from][key_label_to][data_loader.DATA]]
                    durations = [element[data_loader.SEC_DURATION] for element in data_loader.data[0][key_scope][key_label_from][key_label_to][data_loader.DATA]]
                    duration_mean_ms = data_loader.data[0][key_scope][key_label_from][key_label_to][data_loader.SEC_DURATION_MEAN] * 1000.0

                    fig = plt.figure()
                    ax = fig.add_subplot(111)
                    label=key_scope + ' (mean={:.1f} ms)'.format(duration_mean_ms)
                    ax.plot(durations, color='red', label=label)
                    ax.legend()
                    
                    plt.show(block=True)

    else:

        for key_scope in data_loader.data[0]:
            if key_scope not in data_loader.data[1]:
                continue

            for key_label_from in data_loader.data[0][key_scope]:
                if key_label_from not in data_loader.data[1][key_scope]:
                    continue

                for key_label_to in data_loader.data[0][key_scope][key_label_from]:
                    if key_label_to not in data_loader.data[1][key_scope][key_label_from]:
                        continue
                    
                    A_durations = [element[data_loader.SEC_DURATION] for element in data_loader.data[0][key_scope][key_label_from][key_label_to][data_loader.DATA]]
                    B_durations = [element[data_loader.SEC_DURATION] for element in data_loader.data[1][key_scope][key_label_from][key_label_to][data_loader.DATA]]
                    
                    convolve_N = 20
                    A_durations_convolved = np.convolve(A_durations, np.ones(convolve_N)/convolve_N, mode='valid')
                    B_durations_convolved = np.convolve(B_durations, np.ones(convolve_N)/convolve_N, mode='valid')

                    A_duration_mean_ms = data_loader.data[0][key_scope][key_label_from][key_label_to][data_loader.SEC_DURATION_MEAN] * 1000.0
                    B_duration_mean_ms = data_loader.data[1][key_scope][key_label_from][key_label_to][data_loader.SEC_DURATION_MEAN] * 1000.0

                    fig = plt.figure()
                    ax = fig.add_subplot(111)
                    A_label=key_scope + ' (mean={:.1f} ms)'.format(A_duration_mean_ms)
                    B_label=key_scope + ' (mean={:.1f} ms)'.format(B_duration_mean_ms)
                    ax.plot(A_durations, color='red', alpha=0.15)
                    ax.plot(B_durations, color='blue', alpha=0.15)
                    ax.plot(A_durations_convolved, label=A_label, color='red')
                    ax.plot(B_durations_convolved, label=B_label, color='blue')
                    plt.xlabel('Sample (-)')
                    plt.ylabel('Time (s)')
                    ax.legend()
                    
                    plt.show(block=True)
                    
# # #}

if __name__ == "__main__":
    
    path_A, path_B, scope, checkpoints, plot = load_params(sys.argv)
    
    # Load data
    loader = DataLoader()
    loader.load_data(path_A, path_B)

    if not loader.valid():
        print('ERROR: no data were loaded')
        print_help()
        exit(-1)

    process_data(loader, scope=scope, checkpoints=checkpoints)
    print_data(loader, path_A, path_B)

    if plot:
        plot_data(loader, path_A, path_B)
