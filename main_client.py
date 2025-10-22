# main.py

import argparse
from tasks import task1, task4, task5

def main():
    '''Use parser to select and run a lab task'''
    parser = argparse.ArgumentParser()
    parser.add_argument('task', choices=['task1', 'task4', 'task6'])
    args = parser.parse_args()

    if args.task == 'task1':
        print("Running Task 1")
        task1.run()
    elif args.task == 'task4':
        print("Running Task 4")
        task4.run()
    elif args.task == 'task6':
        print("Running Task 6")
        task5.run
if __name__ == '__main__':
    main()
