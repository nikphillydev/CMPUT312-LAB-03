# main.py

import argparse
from tasks import task2, task3

def main():
    '''Use parser to select and run a lab task'''
    parser = argparse.ArgumentParser()
    parser.add_argument('task', choices=['task2', 'task3'])
    args = parser.parse_args()

    if args.task == 'task2':
        print("Running Task 2")
        task2.run()
    elif args.task == 'task3':
        print("Running Task 3")
        task3.run()
if __name__ == '__main__':
    main()
