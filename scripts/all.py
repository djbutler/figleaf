import subprocess
import sys
import itertools

if len(sys.argv) < 2:
    print 'Usage: all.py user_id'
user_id = int(sys.argv[1])

conditions = ['clean', 'mid', 'box']
permutations = [x for x in itertools.permutations(conditions)]

# User IDs start from 1.
conditions = permutations[user_id-1 % len(permutations)]

def confirm(action, callback):
    confirmation = None
    while confirmation is None:
        confirmation = raw_input('Run {}? [y/n]: '.format(action))
        if confirmation == 'y':
            callback()
        elif confirmation == 'n':
            continue
        else:
            confirmation = None

def run_tutorial():
    print 'Running command: ./tutorial.sh'
    subprocess.call(['./tutorial.sh'], shell=True)
def run_experiment(user_id, flight, condition, task):
    cmd = ['./experiment.sh', str(user_id), flight, condition, task]
    print 'Running command: {}'.format(' '.join(cmd))
    subprocess.call(' '.join(cmd), shell=True)

confirm('tutorial', run_tutorial)
for flight, condition in zip(['flight1', 'flight2', 'flight3'], conditions):
    for task in ['func', 'adv']:
        exp = lambda: run_experiment(user_id, flight, condition, task)
        confirm('{} {} {}'.format(flight, condition, task), exp)

print 'All done!'
